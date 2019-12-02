% Destiny Fawley
% 11/6/2019

function nav = navigation(y,tCurr, trajCalcs, models, nav, i)


if mod(i-1,nav.rateRatio) == 0 % rate limit calls to navigation
    switch models.navMode
        case 1 % perfect nav

              nav.imu9250.accel = trajCalcs.accel;
              nav.imu9250.omega = y(10:12);
              nav.imu9250.mag = y(1:3);
              nav.baro.alt = y(3);
            
        case 2 % normal noise distribution
            if mod(i-1,nav.IMURateRatio) == 0 % rate limit calls to barometer
                % noise on IMU acceleration reading
                accelNoise = normrnd(models.IMUAccelNoiseBias,...
                    models.IMUAccelNoiseStd,[3,1]);
                nav.imu9250.accel = trajCalcs.accel + accelNoise;
                
                % noise on IMU gyro reading
                gyroNoise = normrnd(models.IMUGyroNoiseBias,...
                    models.IMUGyroNoiseStd,[3,1]);
                nav.imu9250.omega = y(10:12) + gyroNoise;
                
                % noise on IMU magnetometer reading
                magNoise = normrnd(models.IMUMagNoiseBias,...
                    models.IMUMagNoiseStd,[3,1]);
                nav.imu9250.mag = y(1:3) + magNoise;
            end
            if mod(i-1,nav.MPLRateRatio) == 0 % rate limit calls to barometer
                % noise on barometer
                baroNoise = normrnd(models.MPLAltNoiseBias,...
                    models.MPLAltNoiseStd,1);
                alt = y(3) + baroNoise;
                % limit resolution on altitude reading
                if mod(alt,models.MPLAltResolution) < models.MPLAltResolution/2
                    nav.baro.alt = alt - mod(alt,models.MPLAltResolution);
                else
                    nav.baro.alt = alt - mod(alt,models.MPLAltResolution)+models.MPLAltResolution;
                end
            end
            
    end
end

end





