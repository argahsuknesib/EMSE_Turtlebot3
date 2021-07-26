odom <- read.csv("/home/whiskygrandee/catkin_ws/src/emse_bot/log/feature-generator/odom-23-07-21.csv")

View(odom)

names(odom)
names(odom)[names(odom) == 'seq'] <- 'header.seq'
names(odom)[names(odom) == 'secs'] <- 'stamp.secs'
names(odom)[names(odom) == 'nsecs'] <- 'stamp.nsecs'

names(odom)[names(odom) == 'x'] <- 'position.x'
names(odom)[names(odom) == 'y'] <- 'position.y'
names(odom)[names(odom) == 'z'] <- 'position.z'

names(odom)[names(odom) == 'x.1'] <- 'orientation.x'
names(odom)[names(odom) == 'y.1'] <- 'orientation.y'
names(odom)[names(odom) == 'z.1'] <- 'orientation.z'
names(odom)[names(odom) == 'w'] <- 'orientation.z'

names(odom)[names(odom) == 'x.2'] <- 'linear.x'
names(odom)[names(odom) == 'y.2'] <- 'linear.y'
names(odom)[names(odom) == 'z.2'] <- 'linear.z'

names(odom)[names(odom) == 'x.3'] <- 'linear.x'
names(odom)[names(odom) == 'y.3'] <- 'linear.y'
names(odom)[names(odom) == 'z.3'] <- 'linear.z'

names(odom)
View(odom)

allmisscols <- apply(odom,2, function(x)all(is.na(x)));
colswithallmiss <-names(allmisscols[allmisscols>0]);
o2 <- odom[,  !(names(odom) %in% colswithallmiss)];

allmisscols <- apply(o2,2, function(x)all(x == ''));
colswithallmiss <-names(allmisscols[allmisscols>0]);
o3 <- o2[,  !(names(o2) %in% colswithallmiss)];

names(o3)
View(o3)

allsamecols <- apply(o3,2, function(x) length(unique(x)) ==1);
colswithsame <-names(allsamecols[allsamecols>0]);
colswithsame

ind <- grep('angular', colswithsame);
colswithsame <- colswithsame[-ind];
colswithsame

ind <- grep('linear', colswithsame);
colswithsame <- colswithsame[-ind];
colswithsame

o_final <- o3[,  !(names(o3) %in% colswithsame)];

View(o_final)

