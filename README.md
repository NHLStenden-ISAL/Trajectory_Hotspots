# Trajectory Hotspots

![build workflow](https://github.com/NHLStenden-ISAL/Trajectory_Hotspots/actions/workflows/build_and_test_x64.yml/badge.svg)

This repository contains the implementation of algorithms for hotspot computation on trajectory data. These algorithms are described in a paper by Gudmundsson et al.

> J. Gudmundsson, M. van Kreveld, and F. Staals, “Algorithms for hotspot computation on trajectory data,” in Proceedings of the 21st ACM SIGSPATIAL International Conference on Advances in Geographic Information Systems - SIGSPATIAL’13, 2013, pp. 134–143.

There are four algorithms in total, each of which places different constraints on the trajectory contained within the hotspot. Because of these variations, each case needs a different approach to find the hotspot.