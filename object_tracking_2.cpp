
class Object {
 public:
  Object(Pose2 prior_mean, noiseModel::Diagonal::shared_ptr prior_noise) {
    (0.0, 0.0, 0.0);
    noiseModel::Diagonal::shared_ptr priorNoise =
        graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));
  }

 private:
  std::vector<PriorFactor<Pose2>> factors;
};

class ObjectTracker {
 public:
  ObjectTracker() { noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1)); }

 private:
  NonlinearFactorGraph graph;
  std::vector<Objects> objects;
}

int main(int argc, char* argv)
{
  return 0;
}