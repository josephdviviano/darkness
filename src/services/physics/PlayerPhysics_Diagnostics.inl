// Included inside PlayerPhysics class body — do not include standalone.

    /// Write one CSV row with full physics state. Called at end of fixedStep().
    inline void writeLogRow() {
        if (!mLogFile) return;

        float hSpeed = horizontalSpeed();
        Vector3 eye = computeRawEyePos();

        std::fprintf(mLogFile,
            "%.4f,%.5f,%s,%d,%d,"           // simTime,dt,mode,sneaking,running
            "%.3f,%.3f,%.3f,"               // posX,posY,posZ
            "%.3f,%.3f,%.3f,%.3f,"          // velX,velY,velZ,hSpeed
            "%.4f,%.4f,%.4f,"               // eyeX,eyeY,eyeZ
            "%.4f,%.4f,"                    // yaw,camPitch
            "%.5f,%.5f,%.5f,"              // springPosX,springPosY,springPosZ
            "%.5f,%.5f,%.5f,"              // springVelX,springVelY,springVelZ
            "%.5f,%.5f,%.5f,"              // poseTargetX,poseTargetY,poseTargetZ
            "%.5f,%.5f,%.5f,"              // poseStartX,poseStartY,poseStartZ
            "%.5f,%.5f,%.5f,"              // bodyPoseX,bodyPoseY,bodyPoseZ
            "%.4f,%.4f,%d,"                // poseTimer,poseDur,poseHolding
            "%.3f,%d,%d,%.4f,"              // strideDist,strideIsLeft,leanDir,leanAmount
            "%d,%.2f,%.2f\n",               // cell,inputFwd,inputRight
            mSimTime, mTimestep.fixedDt, modeName(mCurrentMode),
            (int)mSneaking, (int)mRunning,
            mPosition.x, mPosition.y, mPosition.z,
            mVelocity.x, mVelocity.y, mVelocity.z, hSpeed,
            eye.x, eye.y, eye.z,
            mYaw, mCamPitch,
            mSpringPos.x, mSpringPos.y, mSpringPos.z,
            mSpringVel.x, mSpringVel.y, mSpringVel.z,
            mPoseCurrent.x, mPoseCurrent.y, mPoseCurrent.z,
            mPoseStart.x, mPoseStart.y, mPoseStart.z,
            mBodyPoseCurrent.x, mBodyPoseCurrent.y, mBodyPoseCurrent.z,
            mPoseTimer, mPoseDuration, (int)mPoseHolding,
            mStrideDist, (int)mStrideIsLeft, mLeanDir, mLeanAmount,
            mCellIdx, mInputForward, mInputRight);

        // Flush periodically so data isn't lost on crash
        static int flushCounter = 0;
        if (++flushCounter >= 10) {
            std::fflush(mLogFile);
            flushCounter = 0;
        }
    }
