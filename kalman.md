| Bayesian Concept | Kalman Filter Notation      | Explanation (Intuition)                               |
|------------------|-----------------------------|-------------------------------------------------------|
| Prior            | $\hat{x}_{k|k-1}, P_{k|k-1}$| What I believe before seeing data at step $k$.        |
| Likelihood       | $z_k, R_k$                  | How likely is this measurement given my predicted state?|
| Posterior        | $\hat{x}_{k|k}, P_{k|k}$    | What I believe now after incorporating this measurement.|
