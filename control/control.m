% Destiny Fawley
% 11/6/2019

function ctrl = control(motor,rocket,ctrl,nav,tCurr, models, i)

if mod(i-1,ctrl.rateRatio) == 0 % rate limit calls to control
    
    switch models.ctrlMode
        case 1 % state space controller
            
            % get TVC angle commands
            ctrl = stateSpace(motor, rocket, ctrl, nav);
            
            % send ignite motor command
            if ~ctrl.igniteMotor
                if tCurr >= ctrl.tIgnite
                    ctrl.igniteMotor = 1;
                end
            end
            
        otherwise 
            % do nothing
    
    end
              
    
end
end