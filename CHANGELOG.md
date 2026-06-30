# Changelog
All notable changes to this project are documented in this file.

Versions prior to 2.3 were developed outside of version control and are not included in the Git history. They are documented here to provide context and highlight key project milestones.

## [Unreleased]
### Added
* Add stand and crawl walk gait
* Add servo ID change utility (`robokitty servo set-id`)


### Changed
* Uplift ax12a driver
* Modify CLI to support new gait

## [v2.5.0]
### Added
* Build pipeline to generate Python wheel packages
* Release pipeline to publish changelog on tag/release

### Changed
* Modify pre-commit to execute on every push
* Uplift ax12a driver

### Fixed
* Versioning on tag releases

## [v2.4.0]
### Added
* Stream and file based logger
* CI actions for code quality checks

### Changed
* Reorganised code base to fit basic python project

## [v2.3.0]
### Changed
* Increased step length from 40 mm to 80 mm
* Reduced cycle time from 2.4 s to 1.6 s
* Each 20 ms control cycle now moves the femur by 3.6 servo units (previously 0.5 units, below AX-12A compliance threshold and resulting in no torque)
* Set compliance margin to 0 on all servos, removing the 1-unit dead zone where no force is applied
* Increased step height from 18 mm to 25 mm to improve ground clearance

### Fixed
* Corrected movement direction (introduced in version 2.2), ensuring stance phase pushes toward the physical front (-x axis)

## [v2.0.0]
### Added
* Ground press behaviour

### Changed
* Removed simulation mode

## [v1.9.0]
### Added
* Implemented walk (creep) gait
* Body centre-of-mass shifting

## [v1.8.0]
### Added
* Audit logging

### Changed
* Increased gait swing amplitude

## [v1.7.0]
### Changed
* Applied calibrated servo offsets

## [v1.0.0]
### Added
* Initial system implementation
