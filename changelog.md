# Changelog

All notable changes to this project are documented in this file.

Versions prior to 2.3 were developed outside of version control and are not included in the Git history. They are documented here to provide context and highlight key project milestones.

---

## [2.3] - Latest

### Major Changes

* Increased step length from 40 mm to 80 mm

* Reduced cycle time from 2.4 s to 1.6 s

* Each 20 ms control cycle now moves the femur by 3.6 servo units (previously 0.5 units, below AX-12A compliance threshold and resulting in no torque)

* Set compliance margin to 0 on all servos, removing the 1-unit dead zone where no force is applied

* Increased step height from 18 mm to 25 mm to improve ground clearance

### Fixes

* Corrected movement direction (introduced in version 2.2), ensuring stance phase pushes toward the physical front (-x axis)

---

## [2.0]

### Changes

* Added ground press behaviour
* Removed simulation mode

---

## [1.9]

### Features

* Implemented walk (creep) gait
* Added body centre-of-mass shifting

---

## [1.8]

### Improvements

* Increased gait swing amplitude
* Added audit logging

---

## [1.7]

### Changes

* Applied calibrated servo offsets

---

## [1.0]

### Initial Release

* Initial system implementation
