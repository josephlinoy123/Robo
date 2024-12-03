# Benchmarking Simulated Robotic Manipulation through a Real World Dataset

We  present  a  benchmark  to  facilitate  simulated manipulation; an attempt to overcome the obstacles of physical benchmarks  through  the  distribution  of  a  real  world,  groundtruth  dataset.  Users  are  given  various  simulated  manipulationtasks with assigned protocols having the objective of replicatingthe  real  world  results  of  a  recorded  dataset

*[Jack Collins](https://jacktcollins.com), [Jessie McVicar](https://au.linkedin.com/in/jessie-mcvicar-532395180), [David Wedlock](https://au.linkedin.com/in/david-wedlock-53900836), [Ross Brown](https://staff.qut.edu.au/staff/r.brown), [David Howard](https://people.csiro.au/H/D/David-Howard), [Jürgen Leitner](http://juxi.net)*

## Publication

**Benchmarking Simulated Robotic Manipulation through a Real World Dataset**

Robotics and Autonomous Letters Special Issue on Benchmarking Protocols for Robotic Manipulation (RA-L), 2019

[arXiv](https://arxiv.org/abs/1911.01557) | [Website](https://research.csiro.au/robotics/manipulation-benchmark/) | [Datastore](https://doi.org/10.25919/5de4739688946)

<!-- If you use this work, please cite the following as appropriate:

```text
@inproceedings{"Collins2019BenchmarkingDataset", 
	title={{Benchmarking Simulated Robotic Manipulation through a Real World Dataset}}, 
	author={Jack Collins, Jessie McVicar, David Wedlock, Ross Brown, David Howard and J\"urgen Leitner}, 
	journal={IEEE Robotics and Autonomous Letters}, 
	year={2019} 
}

``` -->


## Installation

This code was developed with Python 3.5 on Ubuntu 16.04.

V-REP: Version 3.5.0*

PyBullet: Version 2.1.1

*Please add vrep.py, vrepConst.py and remoteApi.so files relevant for your distribution of V-Rep to the Software directory.

## Packages Overview

* `kinova_description`: Contains external dependencies required to run the simulation including mesh files and the URDF of the robot arm.
* `Results`: Contains the mean results from the dataset and is where the results from the simulations will be saved.
* `BenchmarkingMetrics`: Folder containing the python script to run the benchmarking metrics.
* `main.py`: Must have the filepath for Vrep dependancies updated by the user. 
* `kinova_vr.py`: VRep simulation class.
* `kinova_pb.py`: PyBullet simulation class.
* `PID.py`: Used as the proportional controller.
* `Rotations.py`: Generated target thetas based upon current time step.

## Running

To execute run main.py

## Results and Metrics

The results from execution main.py will be saved in the Results directory. The results are in the form of .csv files. Use the performance_metrics.py script in the BenchmarkingMetrics folder to calculate the metrics for submission in the website.

**Contact**

Any questions or comments contact [Jack Collins](mailto:Jack.Collins@data61.csiro.au).