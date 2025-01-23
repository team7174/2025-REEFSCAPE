#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>
#include <units/length.h>

namespace FieldConstants {

inline constexpr units::meter_t InchesToMeters(units::inch_t inches) {
    return inches;
}

constexpr units::meter_t fieldLength = InchesToMeters(690.876_in);
constexpr units::meter_t fieldWidth = InchesToMeters(317_in);
constexpr units::meter_t startingLineX = InchesToMeters(299.438_in);

namespace Processor {
    constexpr frc::Pose2d centerFace{InchesToMeters(235.726_in), 0_m, frc::Rotation2d{units::degree_t(90)}};
}

namespace Barge {
    constexpr frc::Translation2d farCage{InchesToMeters(345.428_in), InchesToMeters(286.779_in)};
    constexpr frc::Translation2d middleCage{InchesToMeters(345.428_in), InchesToMeters(242.855_in)};
    constexpr frc::Translation2d closeCage{InchesToMeters(345.428_in), InchesToMeters(199.947_in)};

    constexpr units::meter_t deepHeight = InchesToMeters(3.125_in);
    constexpr units::meter_t shallowHeight = InchesToMeters(30.125_in);
}

namespace CoralStation {
    constexpr frc::Pose2d leftCenterFace{InchesToMeters(33.526_in), InchesToMeters(291.176_in), frc::Rotation2d{units::degree_t(-54.011)}};
    constexpr frc::Pose2d rightCenterFace{InchesToMeters(33.526_in), InchesToMeters(25.824_in), frc::Rotation2d{units::degree_t(54.011)}};
}

namespace Reef {
    constexpr frc::Translation2d center{InchesToMeters(176.746_in), InchesToMeters(158.501_in)};
    constexpr units::meter_t faceToZoneLine = InchesToMeters(12_in);
}

enum class ReefLevel {
    L4,
    L3,
    L2,
    L1
};

class ReefHeightProperties {
public:
    constexpr ReefHeightProperties(units::meter_t h, units::degree_t p) : height(h), pitch(p) {}

    units::meter_t height;
    units::degree_t pitch;

    static constexpr ReefHeightProperties Get(ReefLevel heightLevel) {
        switch (heightLevel) {
            case ReefLevel::L4: return {InchesToMeters(72_in), units::degree_t(-90)};
            case ReefLevel::L3: return {InchesToMeters(47.625_in), units::degree_t(-35)};
            case ReefLevel::L2: return {InchesToMeters(31.875_in), units::degree_t(-35)};
            case ReefLevel::L1: return {InchesToMeters(18_in), units::degree_t(0)};
        }
        return {0_m, units::degree_t(0)}; // Default case
    }
};

} // namespace FieldConstants
