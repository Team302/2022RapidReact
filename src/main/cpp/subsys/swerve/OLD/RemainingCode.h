        // If still used, SwerveDrive
        void SetTargetHeading(units::angle::degree_t targetYaw) override;

    private:
        /// @TODO:  Should be in SwerveDrive
        units::angular_velocity::degrees_per_second_t CalcHeadingCorrection
        (
            units::angle::degree_t  targetAngle,
            double                  kP
        );

        /// @TODO:  Should be in SwerveDrive
        void AdjustRotToMaintainHeading
        (
            units::meters_per_second_t&  xspeed,
            units::meters_per_second_t&  yspeed,
            units::radians_per_second_t& rot 
        );        

        /// @TODO:  Should be in SwerveDrive
        void AdjustRotToPointTowardGoal
        (
            frc::Pose2d                  robotPose,
            units::radians_per_second_t& rot
        );

        

        /// @TODO: Remove or try to repurpose for turn about point?
        units::angle::degree_t UpdateForPolarDrive
        (
            frc::Pose2d              robotPose,
            frc::Pose2d              goalPose,
            frc::Transform2d       wheelLoc,
            frc::ChassisSpeeds       speeds
        );
        
        // If anywhere, SwerveChassis/SwerveDrive
        const double                                                m_deadband = 0.0;
        const units::angular_velocity::radians_per_second_t         m_angularDeadband = units::angular_velocity::radians_per_second_t(0.1); //3 degrees per second

        //SwerveDrive
        units::angular_velocity::degrees_per_second_t m_yawCorrection;
    

        // Probably in SwerveSpecial, or SwerveDrive
        DragonTargetFinder m_targetFinder;
        units::angle::degree_t m_targetHeading;
        DragonLimelight*        m_limelight;

        // Probably in SwerveSpecial, or SwerveDrive
        const units::length::inch_t m_shootingDistance = units::length::inch_t(105.0); // was 105.0


};