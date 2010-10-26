function [jerk]=computeJerk(time, acc)
 jerk = diff(acc);


  jerk(length(jerk)+1)=jerk(length(jerk));

  jerk = jerk/(time(2)-time(1));




end