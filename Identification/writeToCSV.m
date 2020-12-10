clear; clc;

load('dataset.mat');
valid_ratio = 0.1;
test_ratio = 0.1;

% split dataset into train, validation and test sets
valid_num = floor(mD*valid_ratio);
test_num = floor(mD*test_ratio);
train_num = mD- valid_num - test_num;

permutation = randperm(mD);
train_index = permutation(1:train_num);
valid_index = permutation(train_num+1:train_num+valid_num);
test_index = permutation(train_num+valid_num+1:end);

x_train = x(:,train_index);
xNext_train = xNext(:,train_index);
u_train = u(:,train_index);

x_valid = x(:,valid_index);
xNext_valid = xNext(:,valid_index);
u_valid = u(:,valid_index);

x_test = x(:,test_index);
xNext_test = xNext(:,test_index);
u_test = u(:,test_index);

% Write train dataset
fid = fopen('train_set.csv','w');
fprintf(fid,'%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n','vx','vy','w','vxNext','vyNext','wNext','Fflx','Ffly','Ffrx','Ffry','Frlx','Frrx');

for i=1:train_num
   fprintf(fid,'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n',x_train(1,i),x_train(2,i),x_train(3,i), ...
       xNext_train(1,i),xNext_train(2,i),xNext_train(3,i), u_train(1,i), u_train(2,i), u_train(3,i), u_train(4,i), u_train(5,i), u_train(6,i));
   i
end
fclose(fid);

% Write validation dataset
fid = fopen('validation_set.csv','w');
fprintf(fid,'%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n','vx','vy','w','vxNext','vyNext','wNext','Fflx','Ffly','Ffrx','Ffry','Frlx','Frrx');

for i=1:valid_num
   fprintf(fid,'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n',x_valid(1,i),x_valid(2,i),x_valid(3,i), ...
       xNext_valid(1,i),xNext_valid(2,i),xNext_valid(3,i), u_valid(1,i), u_valid(2,i), u_valid(3,i), u_valid(4,i), u_valid(5,i), u_valid(6,i));
end
fclose(fid);

% Write test dataset
fid = fopen('test_set.csv','w');
fprintf(fid,'%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n','vx','vy','w','vxNext','vyNext','wNext','Fflx','Ffly','Ffrx','Ffry','Frlx','Frrx');

for i=1:test_num
   fprintf(fid,'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n',x_test(1,i),x_test(2,i),x_test(3,i), ...
       xNext_test(1,i),xNext_test(2,i),xNext_test(3,i), u_test(1,i), u_test(2,i), u_test(3,i), u_test(4,i), u_test(5,i), u_test(6,i));
end
fclose(fid);