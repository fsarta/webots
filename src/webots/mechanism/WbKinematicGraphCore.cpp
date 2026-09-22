// Copyright 1996-2024 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbKinematicGraphCore.hpp"

#include <algorithm>
#include <deque>
#include <queue>
#include <set>
#include <sstream>

namespace wbmechanism {

int jointDof(JointType type) {
  switch (type) {
    case FIXED_JOINT:
      return 0;
    case REVOLUTE_JOINT:
    case PRISMATIC_JOINT:
      return 1;
    case REVOLUTE2_JOINT:
      return 2;
    case SPHERICAL_JOINT:
      return 3;
  }
  return 0;
}

const char *jointTypeName(JointType type) {
  switch (type) {
    case FIXED_JOINT:
      return "SolidJoint";
    case REVOLUTE_JOINT:
      return "HingeJoint";
    case PRISMATIC_JOINT:
      return "SliderJoint";
    case REVOLUTE2_JOINT:
      return "Hinge2Joint";
    case SPHERICAL_JOINT:
      return "BallJoint";
  }
  return "Joint";
}

// ---------------------------------------------------------------- Graph

long Graph::addLink(const std::string &name, bool isStatic) {
  Link l;
  l.id = mLinks.empty() ? 0 : mLinks.back().id + 1;
  l.name = name;
  l.isStatic = isStatic;
  mLinks.push_back(l);
  return l.id;
}

long Graph::addJoint(const std::string &name, JointType type, long parentLinkId, long childLinkId, bool isLoopClosure) {
  Joint j;
  j.id = mJoints.empty() ? 0 : mJoints.back().id + 1;
  j.name = name;
  j.type = type;
  j.parentLinkId = parentLinkId;
  j.childLinkId = childLinkId;
  j.isLoopClosure = isLoopClosure;
  mJoints.push_back(j);
  return j.id;
}

const Link *Graph::link(long id) const {
  for (size_t i = 0; i < mLinks.size(); ++i)
    if (mLinks[i].id == id)
      return &mLinks[i];
  return NULL;
}

const Joint *Graph::joint(long id) const {
  for (size_t i = 0; i < mJoints.size(); ++i)
    if (mJoints[i].id == id)
      return &mJoints[i];
  return NULL;
}

Link *Graph::link(long id) {
  return const_cast<Link *>(static_cast<const Graph *>(this)->link(id));
}

Joint *Graph::joint(long id) {
  return const_cast<Joint *>(static_cast<const Graph *>(this)->joint(id));
}

std::vector<long> Graph::jointsOfLink(long linkId) const {
  std::vector<long> result;
  for (size_t i = 0; i < mJoints.size(); ++i)
    if (mJoints[i].parentLinkId == linkId || mJoints[i].childLinkId == linkId)
      result.push_back(mJoints[i].id);
  return result;
}

void Graph::clear() {
  mLinks.clear();
  mJoints.clear();
}

// ---------------------------------------------------------------- Analysis

namespace {
  struct Adjacency {
    // edge = (neighbor link id, joint id)
    std::map<long, std::vector<std::pair<long, long> > > edges;
    void add(long a, long b, long jointId) {
      edges[a].push_back(std::make_pair(b, jointId));
      edges[b].push_back(std::make_pair(a, jointId));
    }
  };

  Adjacency buildAdjacency(const Graph &graph, int *effectiveJoints, int *effectiveJointDof,
                           std::vector<std::string> *warnings) {
    Adjacency adj;
    if (effectiveJoints)
      *effectiveJoints = 0;
    if (effectiveJointDof)
      *effectiveJointDof = 0;
    std::set<long> ground;
    for (size_t i = 0; i < graph.links().size(); ++i)
      if (graph.links()[i].isStatic)
        ground.insert(graph.links()[i].id);
    for (size_t i = 0; i < graph.joints().size(); ++i) {
      const Joint &j = graph.joints()[i];
      if (j.parentLinkId == j.childLinkId) {
        if (warnings && j.parentLinkId >= 0 && graph.link(j.parentLinkId))
          warnings->push_back("joint '" + j.name + "' connects link '" + graph.link(j.parentLinkId)->name +
                              "' to itself and was ignored");
        continue;
      }
      if (ground.count(j.parentLinkId) && ground.count(j.childLinkId)) {
        if (warnings)
          warnings->push_back("joint '" + j.name + "' connects two grounded links and was ignored");
        continue;
      }
      if (effectiveJoints)
        ++(*effectiveJoints);
      if (effectiveJointDof)
        *effectiveJointDof += jointDof(j.type);
      adj.add(j.parentLinkId, j.childLinkId, j.id);
    }
    return adj;
  }

  // spanning forest via 0-1 BFS: non-loop-closure joints cost 0, loop closures
  // cost 1, so loop closures become chords whenever a pure tree path exists
  void buildSpanningForest(const Graph &graph, const Adjacency &adj,
                           std::map<long, std::pair<long, long> > &parent, std::set<long> &treeJoints) {
    parent.clear();
    treeJoints.clear();
    std::set<long> seen;
    std::map<long, long> dist;
    // seeds: grounded links first, then any remaining link
    std::vector<long> seeds;
    for (size_t i = 0; i < graph.links().size(); ++i)
      if (graph.links()[i].isStatic)
        seeds.push_back(graph.links()[i].id);
    for (size_t i = 0; i < graph.links().size(); ++i)
      seeds.push_back(graph.links()[i].id);

    for (size_t s = 0; s < seeds.size(); ++s) {
      const long root = seeds[s];
      if (seen.count(root))
        continue;
      std::deque<std::pair<long, long> > q;  // (link, cost)
      q.push_back(std::make_pair(root, 0));
      seen.insert(root);
      dist[root] = 0;
      while (!q.empty()) {
        const long n = q.front().first;
        const long cost = q.front().second;
        q.pop_front();
        if (cost > dist[n])
          continue;  // stale entry
        const std::vector<std::pair<long, long> > &list = adj.edges.count(n) ? adj.edges.at(n) :
          *(new std::vector<std::pair<long, long> >());  // avoid throwing on isolated links
        for (size_t i = 0; i < list.size(); ++i) {
          const long next = list[i].first;
          const long jointId = list[i].second;
          const Joint *j = graph.joint(jointId);
          const long nextCost = cost + (j->isLoopClosure ? 1 : 0);
          if (!seen.count(next) || nextCost < dist[next]) {
            seen.insert(next);
            dist[next] = nextCost;
            // replace a previous tree edge of `next` if a cheaper path was found
            std::map<long, std::pair<long, long> >::iterator it = parent.find(next);
            if (it != parent.end())
              treeJoints.erase(it->second.second);
            parent[next] = std::make_pair(n, jointId);
            treeJoints.insert(jointId);
            if (j->isLoopClosure)
              q.push_back(std::make_pair(next, nextCost));
            else
              q.push_front(std::make_pair(next, nextCost));
          }
        }
      }
    }
  }

  void collectCycle(long chordA, long chordB, long chordJointId,
                    const std::map<long, std::pair<long, long> > &parent, Loop &loop) {
    // walk up from chordA to the root collecting joints
    std::set<long> fromA;
    long n = chordA;
    while (parent.count(n)) {
      fromA.insert(n);
      n = parent.at(n).first;
    }
    fromA.insert(n);
    // walk up from chordB until reaching the A-path
    long m = chordB;
    while (!fromA.count(m) && parent.count(m)) {
      loop.jointIds.push_back(parent.at(m).second);
      m = parent.at(m).first;
    }
    // m is the deepest common ancestor on the A-path: complete with the A-side
    long a = chordA;
    while (a != m && parent.count(a)) {
      loop.jointIds.push_back(parent.at(a).second);
      a = parent.at(a).first;
    }
    loop.jointIds.push_back(chordJointId);
  }
}  // namespace

Analysis analyze(const Graph &graph, bool planar) {
  Analysis result;
  result.planar = planar;
  result.jointCount = graph.jointCount();
  result.totalJointDof = 0;
  result.isConnected = true;

  if (graph.isEmpty()) {
    result.mobility = 0;
    result.nonGroundLinkCount = 0;
    result.mobilityClass = STATIC_STRUCTURE;
    return result;
  }

  // gather ground links; if none, elect the lowest id as virtual ground
  std::set<long> ground;
  for (size_t i = 0; i < graph.links().size(); ++i)
    if (graph.links()[i].isStatic)
      ground.insert(graph.links()[i].id);
  if (ground.empty()) {
    ground.insert(graph.links()[0].id);
    result.warnings.push_back("no grounded link: '" + graph.links()[0].name +
                              "' was elected as the base link of the mechanism");
  }

  int nonGround = 0;
  for (size_t i = 0; i < graph.links().size(); ++i)
    if (!ground.count(graph.links()[i].id))
      ++nonGround;
  result.nonGroundLinkCount = nonGround;

  int effectiveJoints = 0;
  int effectiveJointDof = 0;
  for (size_t i = 0; i < graph.joints().size(); ++i)
    result.totalJointDof += jointDof(graph.joints()[i].type);
  Adjacency adj = buildAdjacency(graph, &effectiveJoints, &effectiveJointDof, &result.warnings);

  // connectivity of the non-ground links through joints
  std::set<long> visited;
  {
    std::queue<long> q;
    long start = ground.count(graph.links()[0].id) ? graph.links()[0].id : *ground.begin();
    q.push(start);
    visited.insert(start);
    while (!q.empty()) {
      long n = q.front();
      q.pop();
      const std::vector<std::pair<long, long> > &list = adj.edges[n];
      for (size_t i = 0; i < list.size(); ++i) {
        if (!visited.count(list[i].first)) {
          visited.insert(list[i].first);
          q.push(list[i].first);
        }
      }
    }
  }
  for (size_t i = 0; i < graph.links().size(); ++i)
    if (!visited.count(graph.links()[i].id)) {
      result.isConnected = false;
      result.warnings.push_back("link '" + graph.links()[i].name + "' is not connected to the rest of the mechanism");
    }

  // spanning tree preferring joints which are not loop closures, so that explicit
  // loop-closure joints become chords of the fundamental cycles
  std::map<long, std::pair<long, long> > parent;  // link -> (parent link, joint id)
  std::set<long> treeJoints;
  std::vector<long> chords;  // joint ids which close a cycle
  buildSpanningForest(graph, adj, parent, treeJoints);
  // every joint which is not part of the tree closes a fundamental cycle
  for (size_t i = 0; i < graph.joints().size(); ++i) {
    const Joint &j = graph.joints()[i];
    if (!visited.count(j.parentLinkId) || !visited.count(j.childLinkId))
      continue;  // ignored joint (self or ground-to-ground)
    if (!treeJoints.count(j.id))
      chords.push_back(j.id);
  }

  // fundamental cycles: one per chord
  const int requiredDof = planar ? 3 : 6;
  for (size_t c = 0; c < chords.size(); ++c) {
    const Joint *j = graph.joint(chords[c]);
    Loop loop;
    loop.closingJointId = chords[c];
    loop.requiredDof = requiredDof;
    loop.loopDof = 0;
    collectCycle(j->parentLinkId, j->childLinkId, chords[c], parent, loop);
    for (size_t k = 0; k < loop.jointIds.size(); ++k) {
      const Joint *lj = graph.joint(loop.jointIds[k]);
      loop.loopDof += jointDof(lj->type);
    }
    loop.isGeometricallyDependent = loop.loopDof < requiredDof;
    if (loop.isGeometricallyDependent) {
      std::ostringstream os;
      os << "loop closed by joint '" << j->name << "' has only " << loop.loopDof << " joint DOF (< " << requiredDof
         << "): it requires a special geometric alignment (e.g. a parallelogram) "
         << "and is simulated as a soft constraint";
      result.warnings.push_back(os.str());
    }
    if (!j->isLoopClosure) {
      result.warnings.push_back("joint '" + j->name + "' closes a cycle implicitly: "
                                "consider marking it explicitly as a loop closure");
    }
    result.loops.push_back(loop);
  }

  // Grübler–Kutzbach mobility; grounded links count as a single virtual ground:
  // M = f * (nonGround - J) + sum(f_i), with f = 6 (spatial) or 3 (planar)
  if (planar)
    result.mobility = 3 * (nonGround - effectiveJoints) + effectiveJointDof;
  else
    result.mobility = 6 * (nonGround - effectiveJoints) + effectiveJointDof;

  if (result.mobility < 0)
    result.mobilityClass = OVER_CONSTRAINED;
  else if (result.mobility == 0)
    result.mobilityClass = STATIC_STRUCTURE;
  else
    result.mobilityClass = MECHANISM;

  return result;
}

std::string Analysis::summary() const {
  std::ostringstream os;
  switch (mobilityClass) {
    case OVER_CONSTRAINED:
      os << "Over-constrained (mobility " << mobility << "): redundant constraints";
      break;
    case STATIC_STRUCTURE:
      os << "Rigid structure (mobility 0)";
      break;
    case MECHANISM:
      os << "Mechanism with " << mobility << " DOF";
      break;
  }
  os << " — " << nonGroundLinkCount << " moving links, " << jointCount << " joints, " << loops.size()
     << (loops.size() == 1 ? " loop" : " loops");
  return os.str();
}

// ---------------------------------------------------------------- Layout

Layout computeLayout(const Graph &graph, const LayoutOptions &options) {
  Layout layout;
  if (graph.isEmpty())
    return layout;

  // spanning tree with the same preference as the analysis (tree joints first)
  std::map<long, std::vector<std::pair<long, long> > > children;  // link -> [(child link, joint)]
  std::set<long> seen;
  std::vector<long> roots;
  {
    Adjacency adj = buildAdjacency(graph, NULL, NULL, NULL);
    std::map<long, std::pair<long, long> > parent;
    std::set<long> treeJoints;
    buildSpanningForest(graph, adj, parent, treeJoints);
    for (std::map<long, std::pair<long, long> >::const_iterator it = parent.begin(); it != parent.end(); ++it)
      children[it->second.first].push_back(std::make_pair(it->first, it->second.second));
  }

  // start from grounded links (or the first link) so the base ends up on top
  std::vector<long> seeds;
  for (size_t i = 0; i < graph.links().size(); ++i)
    if (graph.links()[i].isStatic)
      seeds.push_back(graph.links()[i].id);
  if (seeds.empty())
    seeds.push_back(graph.links()[0].id);

  for (size_t s = 0; s < seeds.size(); ++s) {
    if (seen.count(seeds[s]))
      continue;
    roots.push_back(seeds[s]);
  }
  // remaining disconnected links become additional components
  for (size_t i = 0; i < graph.links().size(); ++i)
    if (!seen.count(graph.links()[i].id)) {
      roots.push_back(graph.links()[i].id);
      seen.insert(graph.links()[i].id);
    }

  // layered coordinates (root on top, y grows downwards)
  std::map<long, int> depth;
  std::vector<std::vector<long> > layers;
  std::queue<long> q;
  for (size_t r = 0; r < roots.size(); ++r) {
    q.push(roots[r]);
    depth[roots[r]] = 0;
  }
  while (!q.empty()) {
    long n = q.front();
    q.pop();
    const int d = depth[n];
    if ((int)layers.size() <= d)
      layers.resize(d + 1);
    layers[d].push_back(n);
    const std::vector<std::pair<long, long> > &list = children[n];
    for (size_t i = 0; i < list.size(); ++i) {
      depth[list[i].first] = d + 1;
      q.push(list[i].first);
    }
  }

  // barycenter ordering to reduce crossings (two sweeps)
  std::map<long, double> order;
  for (size_t d = 0; d < layers.size(); ++d)
    for (size_t i = 0; i < layers[d].size(); ++i)
      order[layers[d][i]] = (double)i;
  for (int sweep = 0; sweep < 2; ++sweep) {
    for (size_t d = (sweep == 0 ? 1 : layers.size()); sweep == 0 ? d < layers.size() : d > 0; sweep == 0 ? ++d : --d) {
      const size_t di = sweep == 0 ? d : d - 1;
      std::vector<std::pair<double, long> > keyed;
      for (size_t i = 0; i < layers[di].size(); ++i) {
        const long n = layers[di][i];
        // barycenter of the neighbors in the previous layer
        double sum = 0.0;
        int count = 0;
        const std::vector<long> list = graph.jointsOfLink(n);
        for (size_t k = 0; k < list.size(); ++k) {
          const Joint *j = graph.joint(list[k]);
          const long other = (j->parentLinkId == n) ? j->childLinkId : j->parentLinkId;
          if (depth.count(other) && depth[other] == (int)di - 1) {
            sum += order[other];
            ++count;
          }
        }
        keyed.push_back(std::make_pair(count ? sum / count : order[n], n));
      }
      std::stable_sort(keyed.begin(), keyed.end());
      for (size_t i = 0; i < keyed.size(); ++i) {
        layers[di][i] = keyed[i].second;
        order[keyed[i].second] = (double)i;
      }
    }
  }

  double xCursor = 0.0;  // components are laid out side by side
  for (size_t d = 0; d < layers.size(); ++d) {
    const double width = (layers[d].size() - 1) * options.nodeSpacing;
    for (size_t i = 0; i < layers[d].size(); ++i) {
      const double x = xCursor + i * options.nodeSpacing - 0.5 * width;
      layout.linkPositions[layers[d][i]] = std::make_pair(x, d * options.layerHeight);
    }
  }
  // joint glyphs on the middle of their edge (loop closures keep this position too)
  for (size_t i = 0; i < graph.joints().size(); ++i) {
    const Joint &j = graph.joints()[i];
    std::map<long, std::pair<double, double> >::const_iterator itp = layout.linkPositions.find(j.parentLinkId);
    std::map<long, std::pair<double, double> >::const_iterator itc = layout.linkPositions.find(j.childLinkId);
    if (itp == layout.linkPositions.end() || itc == layout.linkPositions.end())
      continue;
    const std::pair<double, double> &pp = itp->second;
    const std::pair<double, double> &pc = itc->second;
    layout.jointPositions[j.id] = std::make_pair(0.5 * (pp.first + pc.first), 0.5 * (pp.second + pc.second));
  }
  return layout;
}

}  // namespace wbmechanism
