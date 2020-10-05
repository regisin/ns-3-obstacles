# 3D obstacle-compliant mobility models for ns-3

This repository contains ns-3 version 3.23 with the implementation of 3 new mobility models. The models word in 3 dimensions and allow for obstacles to be present in the simulation. The objects are used to as internal boundaries, so when nodes collide with them, they act as if they were bouncing off of the external boundaries of the simulation.

Files of interest:
- `src/mobility/model/obstacle-gauss-markov-mobility-model.{h,cc}`: Gauss-Markov model adapted to comply with obstacles in the scenario.
- `src/mobility/model/random-direction-3d-mobility-model.{h,cc}`: Random Direction adapted for three dimensions and obstacles.
- `src/mobility/model/random-walk-3d-mobility-model.{h,cc}`: Random Walk adapted for three dimensions and obstacles.

Example of usage can be found in `scratch/*` folder.


#### Citation

Regis, Paulo Alexandre, Suman Bhunia, and Shamik Sengupta. "Implementation of 3d obstacle compliant mobility models for uav networks in ns-3." In Proceedings of the Workshop on ns-3, pp. 124-131. 2016.

##### Bibtex
```
@inproceedings{regis2016implementation,
  title={Implementation of 3d obstacle compliant mobility models for uav networks in ns-3},
  author={Regis, Paulo Alexandre and Bhunia, Suman and Sengupta, Shamik},
  booktitle={Proceedings of the Workshop on ns-3},
  pages={124--131},
  year={2016}
}

```
