This document is a declaration of software quality for the `buoy_api_py` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# buoy_api_py Quality Declaration

The package `buoy_api_py` claims to be in the **Quality Level 5** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Quality Categories in REP-2004](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-quality-categories) of the ROS2 developer guide.

## Version Policy [1]

### Version Scheme [1.i]

`buoy_api_py` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning).

### Version Stability [1.ii]

`buoy_api_py` is at an unstable version, i.e. `< 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

All message and service definition files located in `msg` and `srv` directories are considered part of the public API.

### API Stability Within a Released ROS Distribution [1.iv]/[1.vi]

TODO

### ABI Stability Within a Released ROS Distribution [1.v]/[1.vi]

`buoy_api_py` does not contain any C or C++ code and therefore will not affect ABI stability.

## Change Control Process [2]

`buoy_api_py` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#quality-practices).

### Change Requests [2.i]

This package requires that all changes occur through a pull request.

### Contributor Origin [2.ii]

TODO

### Peer Review Policy [2.iii]

Following the recommended guidelines for ROS Core packages, all pull requests must have at least 1 peer review.

### Continuous Integration [2.iv]

TODO

### Documentation Policy [2.v]

TODO

## Documentation

### Feature Documentation [3.i]

`buoy_api_py` has a list of provided [messages and services](README.md).
New messages and services require their own documentation in order to be added.

### Public API Documentation [3.ii]

`buoy_api_py` has embedded API documentation, but it is not currently hosted.

### License [3.iii]

The license for `buoy_api_py` is Apache 2.0, the type is declared in the [package.xml](package.xml) manifest file, and a full copy of the license is in the repository level [LICENSE](LICENSE) file.

There are no source files that are currently copyrighted in this package so files are not checked for abbreviated license statements.

### Copyright Statements [3.iv]

There are no currently copyrighted source files in this package.

## Testing [4]

`buoy_api_py` is a package providing strictly message and service definitions and therefore does not require associated tests and has no coverage or performance requirements.

### Linters and Static Analysis [4.v]

`buoy_api_py` uses and passes all the standard linters and static analysis tools for its generated C++ and Python code to ensure it follows the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#linters-and-static-analysis).

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]/[5.ii]

`buoy_api_py` has the following runtime ROS dependencies, which are at **Quality Level 5**:
* `buoy_interfaces` [QUALITY DECLARATION](https://github.com/osrf/mbari_wec_utils/tree/release/v2.0.0-rc1/buoy_interfaces/QUALITY_DECLARATION.md)

It has several "buildtool" dependencies, which do not affect the resulting quality of the package, because they do not contribute to the public library API.

### Direct Runtime Non-ROS Dependencies [5.iii]

`buoy_api_py` does not have any runtime non-ROS dependencies.

## Platform Support [6]

TODO

## Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).

