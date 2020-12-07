function testK

for Kp=6:1:9
    for Ka=4:1:6
%          for Kb=0.2:0.1:0.4
%             UnicycleToPoseBenchmark(Kp,Ka,Kb);
%               BicycleToPointBenchmark(Kp,Ka);
%             BicycleToPoseBenchmark(Kp,Ka,Kb);
%             BicycleToPath(Kp,Ka);
%          end
    end
end
% UnicycleToPoseBenchmark(15,10,25);
% BicycleToPointBenchmark(25,9)
BicycleToPath(7,5);

% for Kp=14:2:19
%     for Ka=3:2:8
%          for Kb=14:2:19
%             UnicycleToPoseBenchmark(Kp,Ka,Kb);
% %               BicycleToPointBenchmark(Kp,Ka);
% %             BicycleToPoseBenchmark(Kp,Ka,Kb);
% %             BicycleToPath(Kp,Ka);
%          end
%     end
% end
% UnicycleToPoseBenchmark(18,5,16);