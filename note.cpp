
1. C++ queue 基本操作：
    入队，如例：q.push(x); 将x接到队列的末端。
    出队，如例：q.pop(); 弹出队列的第一个元素，注意，并不会返回被弹出元素的值。
    访问队首元素，如例：q.front()，即最早被压入队列的元素。
    访问队尾元素，如例：q.back()，即最后被压入队列的元素。
    判断队列空，如例：q.empty()，当队列空时，返回true。
    访问队列中的元素个数，如例：q.size()

2. C++ pair 基本操作：(类似stl::map。(key:value))
    
    pair<string, int> name_age("Tom", "18");

    typedef pair<string,string> Author;
    Author proust("March","Proust");

    pair<int, double> p1;
    p1 = make_pair(1, 1.2);
    cout << p1.first << p1.second << endl;

3. Eigen
    Eigen::Vector3d ng2{0, 0, 1.0}; //定义及初始化
    Eigen::Vector3d ng1 = g.normalized(); // 归一化。其中，Eigen::Vector3d g.

    //FromTwoVectors:n1转到n2对应的四元数。
    //toRotationMatrix:q装R
    Eigen::Matrix3d R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    //R0满足：R0*ng1 = ng2

    //用R初始化q
    Eigen::Quaterniond q{R};

    //多维vector的初始化
    VectorXd dep_vec(getFeatureCount());









