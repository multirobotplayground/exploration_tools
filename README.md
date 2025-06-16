# [Exploration Tools](#exploration_tools)

Frontier Exploration Utilities for ROS 2

This repository provides core utilities, packages, and nodes for classic multi-robot frontier-based exploration in ROS 2. It includes robust occupancy grid processing, frontier detection, and costmap/c-space generation for safe and efficient autonomous exploration.

<p align="center">
    <img src="./docs/images/frontiers.png" width="500"/>
</p>

## [Nodes](#nodes)

- [FrontierDiscoveryNode](./frontier_exploration/include/FrontierDiscoveryNode.h): Detects and publishes frontiers from occupancy grids.
- [OccupancyGridFilter](./frontier_exploration/include/OccupancyGridFilters.h): (If present) Handles local C-space generation for individual robots.

## [Custom Messages](#custom-messages)

This package uses custom messages for frontiers and robot poses. Make sure to build the package so message headers are generated.

## [Support this Project](#support-this-project)

Support this open-source project.

[![Donate](docs/images/Donate-PayPal-green-usd.png)](https://www.paypal.com/donate/?business=YWAAG5LVWXBQC&no_recurring=0&item_name=Support+Open+Source+mobile+robots+projects+for+search+and+rescue+in+natural+disasters.+Your+donation+can+change+lives%21&currency_code=USD)
[![Donate](docs/images/Donate-PayPal-green-brl.png)](https://www.paypal.com/donate/?business=YWAAG5LVWXBQC&no_recurring=0&item_name=Support+Open+Source+mobile+robots+projects+for+search+and+rescue+in+natural+disasters.+Your+donation+can+change+lives%21&currency_code=BRL)

## [License](#license)

All content from this repository is released under a [GPLv3 license](LICENSE).

Author/Maintainer:

- [Alysson Ribeiro da Silva](https://alysson.thegeneralsolution.com/)

emails:

- <alysson.ribeiro.silva@gmail.com>

## [Bug & Feature Requests](#bug--feature-requests)

Please report bugs and do your requests to add new features through the [Issue Tracker](https://github.com/multirobotplayground/Multi-robot-Intermittent-Rendezvous/issues).
