//
// Created by zhangzuo on 22-8-9.
//
#include "Frame.h"
#include "MapPoint.h"
#include "Tracking.h"

std::mutex m_mappoint;
extern yamlfile yamlFile;

long unsigned int Frame::mnId = 0;
vector<cv::KeyPoint> DistributeOctTree(const vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                       const int &maxX, const int &minY, const int &maxY, const int &N);

Frame::Frame()
{}

Frame::Frame(const Frame &frame)
    :mid(frame.mid),
    mTimeStamp(frame.mTimeStamp),
    mkeyframe_id(frame.mkeyframe_id),
    is_keyframe(frame.is_keyframe),
    mKeys(frame.mKeys),
    unKeys(frame.unKeys),
    mDescriptors(frame.mDescriptors.clone()),
    mFrameImg(frame.mFrameImg.clone()),
    mReferKeyFrameId(frame.mReferKeyFrameId),
    KeyPointisMapPoint(frame.KeyPointisMapPoint),
    mCoviewFrameId(frame.mCoviewFrameId)
{
    //应该需要锁
    m_mappoint.lock();
    this->mvpMapPoint = frame.mvpMapPoint;
    m_mappoint.unlock();
    this->SetPose(frame.mTcw);

}

Frame::Frame(cv::Mat &img, double &time_stamp) {

    mid = mnId++;
    mTimeStamp = time_stamp;
    mFrameImg = img;

    cv::Mat img_gray;
    cv::cvtColor(img,img_gray,CV_BGR2GRAY);

    //提取特征点，就只是用了opencv的函数进行提取的
    ExtractORB(img_gray);
    //特征点去畸变，因为kitti数据集已经去畸变过了，所以这里实际上没有处理
    undistortKeyPoint();
}

void Frame::ExtractORB(cv::Mat &imgGray) {

    if(imgGray.empty())
        return;

    std::vector<cv::KeyPoint> kps;   //临时存放keypoint
    kps.reserve(9000);

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(9000,
                                                            yamlFile.scaleFactor,
                                                            yamlFile.nLevels,
                                                            0,
                                                            0,
                                                            2,
                                                            cv::ORB::HARRIS_SCORE,
                                                            16,
                                                            yamlFile.iniThFAST);
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create(9000,
                                                                  yamlFile.scaleFactor,
                                                                  yamlFile.nLevels,
                                                                  0,
                                                                  0,
                                                                  2,
                                                                  cv::ORB::HARRIS_SCORE,
                                                                  16,
                                                                  yamlFile.iniThFAST);
    detector->detect(imgGray.clone(),kps);

    if(kps.size() < 4000){
        ROS_WARN_STREAM("detector keypoint too less!");
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(9000,yamlFile.scaleFactor,yamlFile.nLevels,0,0,2,cv::ORB::HARRIS_SCORE,16,yamlFile.minThFAST);
        detector->detect(imgGray.clone(),kps);
    }

    //mKeys = kps;
    mKeys = DistributeOctTree(kps,0,imgGray.cols,0,imgGray.rows,yamlFile.nFeature);

    descriptor->compute(imgGray,mKeys,mDescriptors);

    KeyPointisMapPoint.resize(mKeys.size(), false);
//    ROS_INFO_STREAM("mDescriptors: " << mDescriptors.size);
}

void Frame::undistortKeyPoint() {

    if(yamlFile.mDistCoef.at<float>(0) == 0.0){
        //如果k1为0,就证明不需要去畸变了
        unKeys = mKeys;
//        for(int i = 0; i < mKeys.size();i++){
//            this->mfeature2d[i].mkeyPoint = mKeys[i];
//            this->mfeature2d[i].mFrameId = this->mid;
//        }

        return;
    }

    return;
}

void Frame::AddMapPoint(MapPoint *pMP, const int idx) {
    //加上local_map之后，需要添加锁
    auto MapPoint_idx = make_pair(pMP, idx);
    mvpMapPoint.push_back(MapPoint_idx);
}

vector<pair<MapPoint*, int>> Frame::GetmvpMapPoint() {
    //需要锁
    m_mappoint.lock();
    auto MapPoints = mvpMapPoint;
    m_mappoint.unlock();
    return MapPoints;
}

void Frame::EarseAllMapPoint() {
    //需要锁
//    if(mvpMapPoint.empty())
//        return;

    m_mappoint.lock();
    mvpMapPoint.clear();
    m_mappoint.unlock();
}

void Frame::DeleteAllMapPoint()
{
    //锁
//    if(mvpMapPoint.empty())
//        return;

    m_mappoint.lock();
    for(auto mMapPoint : mvpMapPoint)
    {
        delete mMapPoint.first;
//        ROS_WARN_STREAM("delete");
    }
    m_mappoint.unlock();
}



vector<cv::KeyPoint> DistributeOctTree(const vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                       const int &maxX, const int &minY, const int &maxY, const int &N)
{
    // Compute how many initial nodes
    // Step 1 根据宽高比确定初始节点数目
    //计算应该生成的初始节点个数，根节点的数量nIni是根据边界的宽高比值确定的，一般是1或者2
    // ! bug: 如果宽高比小于0.5，nIni=0, 后面hx会报错
    const int nIni = round(static_cast<float>(maxX-minX)/(maxY-minY));

    //一个初始的节点的x方向有多少个像素
    const float hX = static_cast<float>(maxX-minX)/nIni;

    //存储有提取器节点的链表
    list<ExtractorNode> lNodes;

    //存储初始提取器节点指针的vector
    vector<ExtractorNode*> vpIniNodes;

    //重新设置其大小
    vpIniNodes.resize(nIni);

    // Step 2 生成初始提取器节点
    for(int i=0; i<nIni; i++)
    {
        //生成一个提取器节点
        ExtractorNode ni;

        //设置提取器节点的图像边界
        //注意这里和提取FAST角点区域相同，都是“半径扩充图像”，特征点坐标从0 开始
        ni.UL = cv::Point2i(hX*static_cast<float>(i),0);    //UpLeft
        ni.UR = cv::Point2i(hX*static_cast<float>(i+1),0);  //UpRight
        ni.BL = cv::Point2i(ni.UL.x,maxY-minY);		        //BottomLeft
        ni.BR = cv::Point2i(ni.UR.x,maxY-minY);             //BottomRight

        //重设vkeys大小
        ni.vKeys.reserve(vToDistributeKeys.size());

        //将刚才生成的提取节点添加到链表中
        //虽然这里的ni是局部变量，但是由于这里的push_back()是拷贝参数的内容到一个新的对象中然后再添加到列表中
        //所以当本函数退出之后这里的内存不会成为“野指针”
        lNodes.push_back(ni);
        //存储这个初始的提取器节点句柄
        vpIniNodes[i] = &lNodes.back();
    }

    //Associate points to childs
    // Step 3 将特征点分配到子提取器节点中
    for(size_t i=0;i<vToDistributeKeys.size();i++)
    {
        //获取这个特征点对象
        const cv::KeyPoint &kp = vToDistributeKeys[i];
        //按特征点的横轴位置，分配给属于那个图像区域的提取器节点（最初的提取器节点）
        vpIniNodes[kp.pt.x/hX]->vKeys.push_back(kp);
    }

    // Step 4 遍历此提取器节点列表，标记那些不可再分裂的节点，删除那些没有分配到特征点的节点
    // ? 这个步骤是必要的吗？感觉可以省略，通过判断nIni个数和vKeys.size() 就可以吧
    list<ExtractorNode>::iterator lit = lNodes.begin();
    while(lit!=lNodes.end())
    {
        //如果初始的提取器节点所分配到的特征点个数为1
        if(lit->vKeys.size()==1)
        {
            //那么就标志位置位，表示此节点不可再分
            lit->bNoMore=true;
            //更新迭代器
            lit++;
        }
            ///如果一个提取器节点没有被分配到特征点，那么就从列表中直接删除它
        else if(lit->vKeys.empty())
            //注意，由于是直接删除了它，所以这里的迭代器没有必要更新；否则反而会造成跳过元素的情况
            lit = lNodes.erase(lit);
        else
            //如果上面的这些情况和当前的特征点提取器节点无关，那么就只是更新迭代器
            lit++;
    }

    //结束标志位清空
    bool bFinish = false;

    //记录迭代次数，只是记录，并未起到作用
    int iteration = 0;

    //声明一个vector用于存储节点的vSize和句柄对
    //这个变量记录了在一次分裂循环中，那些可以再继续进行分裂的节点中包含的特征点数目和其句柄
    vector<pair<int,ExtractorNode*> > vSizeAndPointerToNode;

    //调整大小，这里的意思是一个初始化节点将“分裂”成为四个
    vSizeAndPointerToNode.reserve(lNodes.size()*4);

    // Step 5 利用四叉树方法对图像进行划分区域，均匀分配特征点
    while(!bFinish)
    {
        //更新迭代次数计数器，只是记录，并未起到作用
        iteration++;

        //保存当前节点个数，prev在这里理解为“保留”比较好
        int prevSize = lNodes.size();

        //重新定位迭代器指向列表头部
        lit = lNodes.begin();

        //需要展开的节点计数，这个一直保持累计，不清零
        int nToExpand = 0;

        //因为是在循环中，前面的循环体中可能污染了这个变量，所以清空
        //这个变量也只是统计了某一个循环中的点
        //这个变量记录了在一次分裂循环中，那些可以再继续进行分裂的节点中包含的特征点数目和其句柄
        vSizeAndPointerToNode.clear();

        // 将目前的子区域进行划分
        //开始遍历列表中所有的提取器节点，并进行分解或者保留
        while(lit!=lNodes.end())
        {
            //如果提取器节点只有一个特征点，
            if(lit->bNoMore)
            {
                // If node only contains one point do not subdivide and continue
                //那么就没有必要再进行细分了
                lit++;
                //跳过当前节点，继续下一个
                continue;
            }
            else
            {
                // If more than one point, subdivide
                //如果当前的提取器节点具有超过一个的特征点，那么就要进行继续分裂
                ExtractorNode n1,n2,n3,n4;

                //再细分成四个子区域
                lit->DivideNode(n1,n2,n3,n4);

                // Add childs if they contain points
                //如果这里分出来的子区域中有特征点，那么就将这个子区域的节点添加到提取器节点的列表中
                //注意这里的条件是，有特征点即可
                if(n1.vKeys.size()>0)
                {
                    //注意这里也是添加到列表前面的
                    lNodes.push_front(n1);

                    //再判断其中子提取器节点中的特征点数目是否大于1
                    if(n1.vKeys.size()>1)
                    {
                        //如果有超过一个的特征点，那么待展开的节点计数加1
                        nToExpand++;

                        //保存这个特征点数目和节点指针的信息
                        vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));

                        //?这个访问用的句柄貌似并没有用到？
                        // lNodes.front().lit 和前面的迭代的lit 不同，只是名字相同而已
                        // lNodes.front().lit是node结构体里的一个指针用来记录节点的位置
                        // 迭代的lit 是while循环里作者命名的遍历的指针名称
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                //后面的操作都是相同的，这里不再赘述
                if(n2.vKeys.size()>0)
                {
                    lNodes.push_front(n2);
                    if(n2.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n3.vKeys.size()>0)
                {
                    lNodes.push_front(n3);
                    if(n3.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n4.vKeys.size()>0)
                {
                    lNodes.push_front(n4);
                    if(n4.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }

                //当这个母节点expand之后就从列表中删除它了，能够进行分裂操作说明至少有一个子节点的区域中特征点的数量是>1的
                // 分裂方式是后加的节点先分裂，先加的后分裂
                lit=lNodes.erase(lit);

                //继续下一次循环，其实这里加不加这句话的作用都是一样的
                continue;
            }//判断当前遍历到的节点中是否有超过一个的特征点
        }//遍历列表中的所有提取器节点

        // Finish if there are more nodes than required features or all nodes contain just one point
        //停止这个过程的条件有两个，满足其中一个即可：
        //1、当前的节点数已经超过了要求的特征点数
        //2、当前所有的节点中都只包含一个特征点
        if((int)lNodes.size()>=N 				//判断是否超过了要求的特征点数
           || (int)lNodes.size()==prevSize)	//prevSize中保存的是分裂之前的节点个数，如果分裂之前和分裂之后的总节点个数一样，说明当前所有的
            //节点区域中只有一个特征点，已经不能够再细分了
        {
            //停止标志置位
            bFinish = true;
        }

            // Step 6 当再划分之后所有的Node数大于要求数目时,就慢慢划分直到使其刚刚达到或者超过要求的特征点个数
            //可以展开的子节点个数nToExpand x3，是因为一分四之后，会删除原来的主节点，所以乘以3
            /**
             * //?BUG 但是我觉得这里有BUG，虽然最终作者也给误打误撞、稀里糊涂地修复了
             * 注意到，这里的nToExpand变量在前面的执行过程中是一直处于累计状态的，如果因为特征点个数太少，跳过了下面的else-if，又进行了一次上面的遍历
             * list的操作之后，lNodes.size()增加了，但是nToExpand也增加了，尤其是在很多次操作之后，下面的表达式：
             * ((int)lNodes.size()+nToExpand*3)>N
             * 会很快就被满足，但是此时只进行一次对vSizeAndPointerToNode中点进行分裂的操作是肯定不够的；
             * 理想中，作者下面的for理论上只要执行一次就能满足，不过作者所考虑的“不理想情况”应该是分裂后出现的节点所在区域可能没有特征点，因此将for
             * 循环放在了一个while循环里面，通过再次进行for循环、再分裂一次解决这个问题。而我所考虑的“不理想情况”则是因为前面的一次对vSizeAndPointerToNode
             * 中的特征点进行for循环不够，需要将其放在另外一个循环（也就是作者所写的while循环）中不断尝试直到达到退出条件。
             * */
        else if(((int)lNodes.size()+nToExpand*3)>N)
        {
            //如果再分裂一次那么数目就要超了，这里想办法尽可能使其刚刚达到或者超过要求的特征点个数时就退出
            //这里的nToExpand和vSizeAndPointerToNode不是一次循环对一次循环的关系，而是前者是累计计数，后者只保存某一个循环的
            //一直循环，直到结束标志位被置位
            while(!bFinish)
            {
                //获取当前的list中的节点个数
                prevSize = lNodes.size();

                //保留那些还可以分裂的节点的信息, 这里是深拷贝
                vector<pair<int,ExtractorNode*> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                //清空
                vSizeAndPointerToNode.clear();

                // 对需要划分的节点进行排序，对pair对的第一个元素进行排序，默认是从小到大排序
                // 优先分裂特征点多的节点，使得特征点密集的区域保留更少的特征点
                //! 注意这里的排序规则非常重要！会导致每次最后产生的特征点都不一样。建议使用 stable_sort
                sort(vPrevSizeAndPointerToNode.begin(),vPrevSizeAndPointerToNode.end());

                //遍历这个存储了pair对的vector，注意是从后往前遍历
                for(int j=vPrevSizeAndPointerToNode.size()-1;j>=0;j--)
                {
                    ExtractorNode n1,n2,n3,n4;
                    //对每个需要进行分裂的节点进行分裂
                    vPrevSizeAndPointerToNode[j].second->DivideNode(n1,n2,n3,n4);

                    // Add childs if they contain points
                    //其实这里的节点可以说是二级子节点了，执行和前面一样的操作
                    if(n1.vKeys.size()>0)
                    {
                        lNodes.push_front(n1);
                        if(n1.vKeys.size()>1)
                        {
                            //因为这里还有对于vSizeAndPointerToNode的操作，所以前面才会备份vSizeAndPointerToNode中的数据
                            //为可能的、后续的又一次for循环做准备
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n2.vKeys.size()>0)
                    {
                        lNodes.push_front(n2);
                        if(n2.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n3.vKeys.size()>0)
                    {
                        lNodes.push_front(n3);
                        if(n3.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n4.vKeys.size()>0)
                    {
                        lNodes.push_front(n4);
                        if(n4.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    //删除母节点，在这里其实应该是一级子节点
                    lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                    //判断是是否超过了需要的特征点数？是的话就退出，不是的话就继续这个分裂过程，直到刚刚达到或者超过要求的特征点个数
                    //作者的思想其实就是这样的，再分裂了一次之后判断下一次分裂是否会超过N，如果不是那么就放心大胆地全部进行分裂（因为少了一个判断因此
                    //其运算速度会稍微快一些），如果会那么就引导到这里进行最后一次分裂
                    if((int)lNodes.size()>=N)
                        break;
                }//遍历vPrevSizeAndPointerToNode并对其中指定的node进行分裂，直到刚刚达到或者超过要求的特征点个数

                //这里理想中应该是一个for循环就能够达成结束条件了，但是作者想的可能是，有些子节点所在的区域会没有特征点，因此很有可能一次for循环之后
                //的数目还是不能够满足要求，所以还是需要判断结束条件并且再来一次
                //判断是否达到了停止条件
                if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
                    bFinish = true;
            }//一直进行nToExpand累加的节点分裂过程，直到分裂后的nodes数目刚刚达到或者超过要求的特征点数目
        }//当本次分裂后达不到结束条件但是再进行一次完整的分裂之后就可以达到结束条件时
    }// 根据兴趣点分布,利用4叉树方法对图像进行划分区域

    // Retain the best point in each node
    // Step 7 保留每个区域响应值最大的一个兴趣点
    //使用这个vector来存储我们感兴趣的特征点的过滤结果
    vector<cv::KeyPoint> vResultKeys;

    //调整容器大小为要提取的特征点数目
    vResultKeys.reserve(N);

    //遍历这个节点链表
    for(list<ExtractorNode>::iterator lit=lNodes.begin(); lit!=lNodes.end(); lit++)
    {
        //得到这个节点区域中的特征点容器句柄
        vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;

        //得到指向第一个特征点的指针，后面作为最大响应值对应的关键点
        cv::KeyPoint* pKP = &vNodeKeys[0];

        //用第1个关键点响应值初始化最大响应值
        float maxResponse = pKP->response;

        //开始遍历这个节点区域中的特征点容器中的特征点，注意是从1开始哟，0已经用过了
        for(size_t k=1;k<vNodeKeys.size();k++)
        {
            //更新最大响应值
            if(vNodeKeys[k].response>maxResponse)
            {
                //更新pKP指向具有最大响应值的keypoints
                pKP = &vNodeKeys[k];
                maxResponse = vNodeKeys[k].response;
            }
        }

        //将这个节点区域中的响应值最大的特征点加入最终结果容器
        vResultKeys.push_back(*pKP);
    }

    //返回最终结果容器，其中保存有分裂出来的区域中，我们最感兴趣、响应值最大的特征点
    return vResultKeys;
}

void ExtractorNode::DivideNode(ExtractorNode &n1,
                               ExtractorNode &n2,
                               ExtractorNode &n3,
                               ExtractorNode &n4)
{
    //得到当前提取器节点所在图像区域的一半长宽，当然结果需要取整
    const int halfX = ceil(static_cast<float>(UR.x-UL.x)/2);
    const int halfY = ceil(static_cast<float>(BR.y-UL.y)/2);

    //Define boundaries of childs
    //下面的操作大同小异，将一个图像区域再细分成为四个小图像区块
    //n1 存储左上区域的边界
    n1.UL = UL;
    n1.UR = cv::Point2i(UL.x+halfX,UL.y);
    n1.BL = cv::Point2i(UL.x,UL.y+halfY);
    n1.BR = cv::Point2i(UL.x+halfX,UL.y+halfY);
    //用来存储在该节点对应的图像网格中提取出来的特征点的vector
    n1.vKeys.reserve(vKeys.size());

    //n2 存储右上区域的边界
    n2.UL = n1.UR;
    n2.UR = UR;
    n2.BL = n1.BR;
    n2.BR = cv::Point2i(UR.x,UL.y+halfY);
    n2.vKeys.reserve(vKeys.size());

    //n3 存储左下区域的边界
    n3.UL = n1.BL;
    n3.UR = n1.BR;
    n3.BL = BL;
    n3.BR = cv::Point2i(n1.BR.x,BL.y);
    n3.vKeys.reserve(vKeys.size());

    //n4 存储右下区域的边界
    n4.UL = n3.UR;
    n4.UR = n2.BR;
    n4.BL = n3.BR;
    n4.BR = BR;
    n4.vKeys.reserve(vKeys.size());

    //Associate points to childs
    //遍历当前提取器节点的vkeys中存储的特征点
    for(size_t i=0;i<vKeys.size();i++)
    {
        //获取这个特征点对象
        const cv::KeyPoint &kp = vKeys[i];
        //判断这个特征点在当前特征点提取器节点图像的哪个区域，更严格地说是属于那个子图像区块
        //然后就将这个特征点追加到那个特征点提取器节点的vkeys中
        //NOTICE BUG REVIEW 这里也是直接进行比较的，但是特征点的坐标是在“半径扩充图像”坐标系下的，而节点区域的坐标则是在“边缘扩充图像”坐标系下的
        if(kp.pt.x<n1.UR.x)
        {
            if(kp.pt.y<n1.BR.y)
                n1.vKeys.push_back(kp);
            else
                n3.vKeys.push_back(kp);
        }
        else if(kp.pt.y<n1.BR.y)
            n2.vKeys.push_back(kp);
        else
            n4.vKeys.push_back(kp);
    }//遍历当前提取器节点的vkeys中存储的特征点

    //判断每个子特征点提取器节点所在的图像中特征点的数目（就是分配给子节点的特征点数目），然后做标记
    //这里判断是否数目等于1的目的是确定这个节点还能不能再向下进行分裂
    if(n1.vKeys.size()==1)
        n1.bNoMore = true;
    if(n2.vKeys.size()==1)
        n2.bNoMore = true;
    if(n3.vKeys.size()==1)
        n3.bNoMore = true;
    if(n4.vKeys.size()==1)
        n4.bNoMore = true;
}


