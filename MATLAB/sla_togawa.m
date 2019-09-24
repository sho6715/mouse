close all

DEG_TO_RAD = pi()/180;
%à¯êî
f_speed = 500;
f_angAcc = 500.0;
f_g =4000;
enSLA_TYPE = 'SLA_90';
%äpìxÇ…âûÇ∂ÇƒïœçX
f_start_x = 0;
f_start_y = 0;
f_final_x = 0;
f_final_y = 0;
f_final_ang = 0;
%%
f_maxAngleV =0;
f_timeAcc = 0;
f_accAngle = 0;
f_timeConst =0;
f_constAngle = 0;
f_ang = 0;
f_time = 0;
f_x = 0;
f_y = 0;

switch(enSLA_TYPE)
    case 'SLA_90'
        f_start_x = 90;
        f_start_y = 0.0;
        f_final_x = 180;
        f_final_y = 90;
        f_final_ang = 90.0 * DEG_TO_RAD;

    case 'SLA_45'
        f_start_x = 90;
        f_start_y = 0.0;
        f_final_x = 180*0.75;
        f_final_y = 180*0.75;
        f_final_ang = 45.0 * DEG_TO_RAD;

    case 'SLA_N90'
			f_start_x   = 90 * 0.5 * 1.4142;
			f_start_y   = 0.0;
			f_final_x   = 90 * 1.4142;
			f_final_y   = 90 * 0.5 * 1.4142;
			f_final_ang = 90.0 * DEG_TO_RAD;
			
	case 'SLA_135'
			f_start_x   = 90;
			f_start_y   = 0.0;
			f_final_x   = 180 * 1.25;
			f_final_y   = 180 * 0.25;
			f_final_ang = 135.0 * DEG_TO_RAD;
end

f_maxAngleV = f_g/f_speed;
f_timeAcc = f_maxAngleV / f_angAcc;
f_accAngle = 0.5 * f_angAcc * f_timeAcc * f_timeAcc;
f_constAngle = f_final_ang - f_accAngle * 2;
f_timeConst = f_constAngle / f_maxAngleV;

f_x = f_start_x;
f_y = f_start_y;

%figure();

for i = 1:f_timeAcc*1000
    f_time = 0.001 * i;
    f_ang = 0.5 * f_angAcc * f_time * f_time;
    f_x = f_x + f_speed * sin(f_ang) * 0.001;
    f_y = f_y + f_speed * cos(f_ang) * 0.001;
    %ï`âÊèàóù
 %   plot(f_x,f_y,'.b');hold on 
end

for i = 1:f_timeConst*1000
    f_time = 0.001 * i;
    f_ang = f_accAngle + f_maxAngleV * f_time;
    f_x = f_x + f_speed * sin(f_ang) * 0.001;
    f_y = f_y + f_speed * cos(f_ang) * 0.001;
    %ï`âÊèàóù
%    plot(f_x,f_y,'.r'); hold on
end

for i = 1:f_timeAcc*1000
    f_time = 0.001 * i;
    f_ang = f_accAngle + f_constAngle + 0.5 * f_angAcc * f_time * f_time;
    f_x = f_x + f_speed * sin(f_ang) * 0.001;
    f_y = f_y + f_speed * cos(f_ang) * 0.001;
    %ï`âÊèàóù
%    plot(f_x,f_y,'.g'); hold on
end

%xlim([0 180])
%ylim([0 180])

switch(enSLA_TYPE)
    case 'SLA_90'
        f_escapeLen = f_final_x - f_x;
        f_entryLen = f_final_y - f_y;
        
    case 'SLA_45'
        f_escapeLen = 1.4142 * (f_final_x - f_x);
        f_entryLen = f_final_y - f_y - (f_final_x - f_x);
        
    case 'SLA_N90'
        f_escapeLen = f_final_x - f_x;
		f_entryLen  = f_final_y - f_y;

    case 'SLA_135'
			f_escapeLen = 1.4142 * ( f_final_x - f_x );
			f_entryLen  = f_final_y - f_y + ( f_final_x - f_x );

end

f_x = f_start_x;
f_y = f_start_y+f_entryLen;

figure(); hold on;
plot([f_start_x f_start_x],[f_start_y f_start_y+f_entryLen],'LineWidth',2);
for i = 1:f_timeAcc*1000
    f_time = 0.001 * i;
    f_ang = 0.5 * f_angAcc * f_time * f_time;
    f_x = f_x + f_speed * sin(f_ang) * 0.001;
    f_y = f_y + f_speed * cos(f_ang) * 0.001;
    %ï`âÊèàóù
    plot(f_x,f_y,'.g');hold on 
end

for i = 1:f_timeConst*1000
    f_time = 0.001 * i;
    f_ang = f_accAngle + f_maxAngleV * f_time;
    f_x = f_x + f_speed * sin(f_ang) * 0.001;
    f_y = f_y + f_speed * cos(f_ang) * 0.001;
    %ï`âÊèàóù
    plot(f_x,f_y,'.c'); hold on
end

for i = 1:f_timeAcc*1000
    f_time = 0.001 * i;
    f_ang = f_accAngle + f_constAngle + 0.5 * f_angAcc * f_time * f_time;
    f_x = f_x + f_speed * sin(f_ang) * 0.001;
    f_y = f_y + f_speed * cos(f_ang) * 0.001;
    %ï`âÊèàóù
    plot(f_x,f_y,'.r'); hold on
end
plot([f_x f_final_x],[f_y f_final_y],'LineWidth',2);

xlim([0 270])
ylim([0 180])

        