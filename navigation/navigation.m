% Destiny Fawley
% 11/6/2019

function nav = navigation(y,tCurr, trajCalcs, models, nav, i)


if mod(i-1,nav.rateRatio) == 0 % rate limit calls to navigation
    switch models.navMode
        case 1 % perfect nav
            nav.posI = y(1:3);
            nav.velI = y(4:6);
            nav.EulerAngles = y(7:9);
            nav.omega = y(10:12);
            
    end
end

end





