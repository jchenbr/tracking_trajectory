#include "octomap.h"
#include "voxelmacro.h"

#include <ros/console.h>
#include <string.h>
#include <fstream>
#include <list>
#include <vector>
#include <iostream>

namespace VoxelTrajectory
{
    const static int _TAG_NUL   = 0;
    const static int _TAG_EMP   = 1;
    const static int _TAG_OBS   = 2;
    
    const static int _TAG_MIX   = 3;
    const static int _TAG_TAL   = 4;

    const static int _NODE_NULL = 0;
    const static int _NODE_ROOT = 1;
    using namespace std;
    const double eps = 1e-9;
    const double _EPS = 1e-11;

    OctoMap::Node::Node()
    {
    }

    OctoMap::Node::Node(const double bdy[_TOT_BDY], const int son[_TOT_CHD], int id, int tag)
    {
        assert(_TAG_NUL <= tag);
        assert(tag < _TAG_TAL);

        this->id    = id;

        if (bdy == NULL)
        {
            memset(this->bdy, 0, sizeof(this->bdy));
        }
        else
        {
            memcpy(this->bdy, bdy, sizeof(this->bdy));
        }

        if (son == NULL)
        {
            memset(this->son, 0, sizeof(this->son));
        }
        else
        {
            memcpy(this->son, son, sizeof(this->son));
        }

        this->tag   = tag;
    }

    void OctoMap::addNode(const double bdy[_TOT_BDY], const int son[_TOT_CHD], int tag)
    {
        node.push_back(OctoMap::Node(bdy, son, (int)node.size(), tag));
    }

    OctoMap::OctoMap(std::string filename)
    {
        this->loadFromFile(filename);
    }

    OctoMap::OctoMap(const double bdy[_TOT_BDY], double resolution)
    {
        // it must be a legal 3-D space;
        assert
            (
                (bdy[_BDY_x] < bdy[_BDY_X]) && 
                (bdy[_BDY_y] < bdy[_BDY_Y]) &&
                (bdy[_BDY_z] < bdy[_BDY_Z])
            );
        //clog << "?????????????????????????????????????????????????????????" << endl;

        // this is the allowed minimal grid volume
        this->resolution    = resolution;

        // NULL Node
        addNode(NULL, NULL, _TAG_NUL);
        node[_NODE_NULL].bdy[_BDY_X] -= eps;
        node[_NODE_NULL].bdy[_BDY_Y] -= eps;
        node[_NODE_NULL].bdy[_BDY_Z] -= eps;

        log.reserve(10000);

        // The big picture, set as tree root
        addNode(bdy, NULL, _TAG_EMP);
        log.push_back(_NODE_ROOT);
        dealWithLog();

        atom.resize(_TOT_DIM);
        atom[_DIM_x] = bdy[_BDY_X] - bdy[_BDY_x];
        atom[_DIM_y] = bdy[_BDY_Y] - bdy[_BDY_y];
        atom[_DIM_z] = bdy[_BDY_Z] - bdy[_BDY_z];

        while (atom[_DIM_x] * atom[_DIM_y] * atom[_DIM_z] > resolution)
        {
            atom[_DIM_x] *= 0.5;
            atom[_DIM_y] *= 0.5;
            atom[_DIM_z] *= 0.5;
        }

    }

    inline bool OctoMap::isAtom(const Node & node)
    {
        return (node.bdy[_BDY_X] - node.bdy[_BDY_x] < this->atom[_DIM_x] + eps);
    }

    OctoMap::~OctoMap()
    {
        for (size_t idx = 0; idx < node.size(); ++idx)
        {
            if (isAtom(node[idx]) && graph_node_ptr[idx] != NULL) 
                VoxelGraph::delNode(graph_node_ptr[idx]);
        }
    }

    static inline bool within(
        const double pt[_TOT_DIM],
        const double bdy[_TOT_BDY])
    {
        return 
            bdy[_BDY_x] < pt[_DIM_x] + eps && pt[_DIM_x] < bdy[_BDY_X] &&
            bdy[_BDY_y] < pt[_DIM_y] + eps && pt[_DIM_y] < bdy[_BDY_Y] &&
            bdy[_BDY_z] < pt[_DIM_z] + eps && pt[_DIM_z] < bdy[_BDY_Z];
    }

    static inline double getVolume(const double bdy[_TOT_BDY])
    {
        return 
            (bdy[_BDY_X] - bdy[_BDY_x]) *
            (bdy[_BDY_Y] - bdy[_BDY_y]) *
            (bdy[_BDY_Z] - bdy[_BDY_z]);
    }

    static inline double max(double a,double b)
    {return (a>b)?a:b;}

    static inline double min(double a,double b)
    {return (a<b)?a:b;}

    static inline void retSharedArea(
        const double bdy_a[_TOT_BDY],
        const double bdy_b[_TOT_BDY],
        double bdy[_TOT_BDY])
    {
        bdy[_BDY_x] = max(bdy_a[_BDY_x], bdy_b[_BDY_x]);
        bdy[_BDY_X] = min(bdy_a[_BDY_X], bdy_b[_BDY_X]);

        bdy[_BDY_y] = max(bdy_a[_BDY_y], bdy_b[_BDY_y]);
        bdy[_BDY_Y] = min(bdy_a[_BDY_Y], bdy_b[_BDY_Y]);

        bdy[_BDY_z] = max(bdy_a[_BDY_z], bdy_b[_BDY_z]);
        bdy[_BDY_Z] = min(bdy_a[_BDY_Z], bdy_b[_BDY_Z]);
    }

    static inline bool isHot(const double bdy[_TOT_BDY])
    {
        return (
            bdy[_BDY_x] < bdy[_BDY_X]  &&
            bdy[_BDY_y] < bdy[_BDY_Y]  &&
            bdy[_BDY_z] < bdy[_BDY_Z] );
    }

    static bool isIntersected(
        const double box[_TOT_BDY],
        const double bdy[_TOT_BDY])
    {
        double tmp[_TOT_BDY];

        retSharedArea(box, bdy, tmp);

        return isHot(tmp);
    }

    static bool testEnclose(
        const double box[_TOT_BDY],
        const double bdy[_TOT_BDY])
    {
        return box[_BDY_x] < bdy[_BDY_x] + eps && bdy[_BDY_X] < box[_BDY_X] + eps &&
               box[_BDY_y] < bdy[_BDY_y] + eps && bdy[_BDY_Y] < box[_BDY_Y] + eps &&
               box[_BDY_z] < bdy[_BDY_z] + eps && bdy[_BDY_Z] < box[_BDY_Z] + eps;
    }

   void OctoMap::insertPoint(const double pt[_TOT_DIM], int rt)
    {
        if (rt == _NODE_NULL) return ;
        if (!within(pt, node[rt].bdy)) return ;

        node[rt].tag |= _TAG_OBS;

        if (isAtom(node[rt]))
        {
            if (node[rt].tag == _TAG_EMP) log.push_back(-rt);
            node[rt].tag = _TAG_OBS;
            return ;
        }

        splitNode(rt);
        
        for (size_t chd = 0; chd < _TOT_CHD; ++chd)
            insertPoint(pt, node[rt].son[chd]);

        update(rt);
    }

    void OctoMap::insertPoints(const vector<double> & pt)
    {
        assert(pt.size() % _TOT_DIM == 0);
        
        log.clear();

        // insert
        for (size_t idx = 0; idx < pt.size(); idx += _TOT_DIM)
        {
            insertPoint(pt.data() + idx, _NODE_ROOT);
        }

        // delete graph node
        
        // add graph node
        dealWithLog();
    }

    void printBlock(const double bdy[_TOT_BDY])
    {
        clog << bdy[0] << ", " << bdy[1] << ", " << bdy[2] << ", ";
        clog << bdy[3] << ", " << bdy[4] << ", " << bdy[5] << endl;
    }

    void OctoMap::insertBlock(const double bdy[_TOT_BDY], int rt)
    {
        if (rt == _NODE_NULL) return;
        //clog << "inserting : " ; printBlock(bdy);
        //clog << " to :" ; printBlock(node[rt].bdy);
        if (!isIntersected(bdy, node[rt].bdy)) return ;

        if (testEnclose(bdy, node[rt].bdy) || isAtom(node[rt]))
        {
            if (node[rt].tag == _TAG_EMP) log.push_back(-rt);
            node[rt].tag    = _TAG_OBS;
            return ;
        }

        splitNode(rt);

        for (size_t chd = 0; chd < _TOT_CHD; ++chd)
            insertBlock(bdy, node[rt].son[chd]);

        update(rt);
    }

    void OctoMap::insertBlocks(const vector<double> & blk)
    {
        assert(blk.size() % _TOT_BDY == 0);

        log.clear();

        // insert graph node
        for (size_t idx = 0; idx < blk.size(); idx += _TOT_BDY)
        {
            insertBlock(blk.data() + idx, _NODE_ROOT);
        }

        //clog << "going to deal with log." << endl;

        // delete graph node
        
        // add graph node
        
        dealWithLog();

        //clog << "Dealed log." << endl;
    }

#if 0
    void OctoMap::insertBlock(const double bdy[_TOT_BDY])
    {
        assert(
                (bdy[_BDY_x] < bdy[_BDY_X]) && 
                (bdy[_BDY_y] < bdy[_BDY_Y]) && 
                (bdy[_BDY_z] < bdy[_BDY_Z])
              );

        insertBlock(bdy, _NODE_ROOT);
    }

    void OctoMap::insertBlock(const double bdy[_TOT_BDY], int rt)
    {
#if 0
        clog<<"<"<< bdy[0] << ", " << bdy[1] << ", " << bdy[2] 
            << ", " << bdy[3] << ", " << bdy[4] << ", " << bdy[5]<<">"<<endl;

        clog<<"<"<< node[rt].bdy[0] << ", " << node[rt].bdy[1] << ", " << node[rt].bdy[2] 
            << ", " << node[rt].bdy[3] << ", " << node[rt].bdy[4] << ", " << node[rt].bdy[5]<<">"<<endl;
        clog<< "{" << isIntersected(bdy, node[rt].bdy) << ", " << Contained(bdy, node[rt].bdy)
            << ", " << (getVolume(node[rt].bdy) < this->resolution + eps)<<" }"<<endl;
#endif
        if (rt == _NODE_NULL) return;
        if (!isIntersected(bdy, node[rt].bdy)) return ;

        splitNode(rt);

        node[rt].tag |= _TAG_OBS;

        if (testEnclose(bdy, node[rt].bdy) || isAtom(node[rt]))
        {
            node[rt].tag    = _TAG_OBS;
            return ;
        }

        for (int chd = 0; chd < _TOT_CHD; chd++)
            insertBlock(bdy, node[rt].son[chd]);

        update(rt);
    }
#endif

#if 0
    static bool PointWithinSphere(
        const double p[_TOT_DIM],
        const double pt[_TOT_DIM], const double r)
    {
        double tmp  =   (p[_DIM_x] - pt[_DIM_x]) * (p[_DIM_x] - pt[_DIM_x]) + 
                        (p[_DIM_y] - pt[_DIM_y]) * (p[_DIM_y] - pt[_DIM_y]) +   
                        (p[_DIM_z] - pt[_DIM_z]) * (p[_DIM_z] - pt[_DIM_z]);
        return sqrt(tmp) < r + eps;
    }

    static bool isSphereContained(
        const double pt[_TOT_DIM], const double r, 
        const double bdy[_TOT_BDY])
    {
        return true;
    }

    static bool SphereContained(
        const double pt[_TOT_DIM], const double r,
        const double bdy[_TOT_DIM])
    {
        bool ret = true;

        for (double mask = 0; msk < (1<<3); msk++)
        {
            double p[] = {bdy[ 0 | (msk&1)], 
                          bdy[ 2 | ((msk>>1) & 1)] ,
                          bdy[ 4 | ((msg>>2) & 1)]};
            ret = ret && PointWithinSphere(p, pt, r);
        }

        return ret;
    }

    void OctoMap::insertSphere(const double pt[_TOT_DIM], const double r, int rt)
    {
        if (rt == _NODE_NULL) return ;
        if (!isSphereIntersected(pt, r, node[rt].bdy)) return ;

        node[rt].tag    |= _TAG_OBS;

        if (SphereContained(pt, r, node[rt].bdy) ||
            getVolume(node[rt].bdy) + eps > this->resolution)
        {
            node[rt].tag    = _TAG_OBS;
        }

        splitNode(rt);

        for (int chd = 0; chd < _TOT_CHD; chd++)
            insertSphere(pt, r, node[rt].son[chd]);

        update(rt);
    }
#endif

    // split the node into 8 subnode;
    void OctoMap::splitNode(int rt)
    {
        if (node[rt].son[0] || isAtom(node[rt])) return;

        log.push_back( -rt );

        double * bdy = node[rt].bdy;
        double BDY[_TOT_BDY][2] = 
        {
            bdy[_BDY_x], (bdy[_BDY_x] + bdy[_BDY_X]) * 0.5, 
            (bdy[_BDY_x] + bdy[_BDY_X]) * 0.5, bdy[_BDY_X],
            bdy[_BDY_y], (bdy[_BDY_y] + bdy[_BDY_Y]) * 0.5,
            (bdy[_BDY_y] + bdy[_BDY_Y]) * 0.5, bdy[_BDY_Y],
            bdy[_BDY_z], (bdy[_BDY_z] + bdy[_BDY_Z]) * 0.5, 
            (bdy[_BDY_z] + bdy[_BDY_Z]) * 0.5, bdy[_BDY_Z]
        };

        for (int chd = 0; chd < _TOT_CHD; ++chd)
        {
            double new_bdy[_TOT_BDY] =
            {   
                BDY[_BDY_x][(chd >> _DIM_x) & 1], BDY[_BDY_X][(chd >> _DIM_x) & 1],
                BDY[_BDY_y][(chd >> _DIM_y) & 1], BDY[_BDY_Y][(chd >> _DIM_y) & 1],
                BDY[_BDY_z][(chd >> _DIM_z) & 1], BDY[_BDY_Z][(chd >> _DIM_z) & 1],
            };
            node[rt].son[chd] = (int)node.size();
            log.push_back((int) node.size());
            
            addNode(new_bdy, NULL, node[rt].tag);
        }
    }

    void OctoMap::connectTo(int rt, int src, const double bdy[_TOT_BDY])
    {
        /*
        if (src == 10)
        {
            clog << "src bdy : "; printBlock(bdy);
            clog << "to  bdy : "; printBlock(node[rt].bdy);
            clog << " tag = " << node[rt].tag << endl;
        }
        */

        if (!isIntersected(node[rt].bdy, bdy)) return ;

        if (node[rt].tag == _TAG_MIX)
        {
            for (size_t chd = 0; chd < _TOT_CHD; ++chd)
            {
                //if (src == 10) clog << "rt = " << rt << ", chd = " << chd << endl;
                connectTo(node[rt].son[chd], src, bdy);
            }
        }
        else if (node[rt].tag == _TAG_EMP)
        {
            double bdy_i[_TOT_BDY];
            retSharedArea(node[rt].bdy, bdy, bdy_i);
            //if (src == 10){clog << "src = " << src << ", to = " << rt << endl;}
            VoxelGraph::addEdge(graph_node_ptr[rt], graph_node_ptr[src], bdy_i);
        }
    }

    void OctoMap::dealWithLog()
    {
        if (log.empty()) return ;
        int old_id = (int) graph_node_ptr.size();
        graph_node_ptr.resize(node.size());

        /*
        clog << "id list : \n";
        for (auto id : log) clog << id << ", ";
        clog << endl;
        */

        for (auto id : log)
        {
            if (id < 0 && (-id) < old_id)
            { 
                if (graph_node_ptr[-id] == NULL) continue;
                VoxelGraph::delNode(graph_node_ptr[-id]);
                graph_node_ptr[-id] = NULL;
            }

            if (id > 0 && node[id].tag == _TAG_EMP) 
            {
                graph_node_ptr[id] = VoxelGraph::addNode(id, node[id].bdy);
            }
        }

        double bdy[_TOT_BDY];
        for (auto id : log)
        {
            if (id > 0 && node[id].tag == _TAG_EMP) 
            {
                for (int dim = 0; dim < _TOT_BDY; dim += 2)
                {
                    memcpy(bdy, node[id].bdy, sizeof(double) * _TOT_BDY);

                    bdy[dim] = bdy[dim | 1] = node[id].bdy[dim];
                    bdy[dim] -= eps;
                    connectTo(_NODE_ROOT, id, bdy);

                    bdy[dim] = bdy[dim | 1] = node[id].bdy[dim | 1];
                    bdy[dim | 1] += eps;
                    connectTo(_NODE_ROOT, id, bdy);
                }
            }
        }

        log.clear();
    }

    // update the tag of the node #rt;
    void OctoMap::update(int rt)
    {
#if 0
        for (int chd: node[rt].son)
        {
            node[rt].tag |= node[chd].tag;
        }
#else       
        int * son = node[rt].son;
        node[rt].tag |= 
            node[son[0]].tag | node[son[1]].tag | 
            node[son[2]].tag | node[son[3]].tag |
            node[son[4]].tag | node[son[5]].tag |
            node[son[6]].tag | node[son[7]].tag;
#endif
    }


#if 0
    void OctoMap::query(int rt,int from, const double bdy[_TOT_BDY],
        VoxelGraph * graph)
    {
#if 0
            if (from < 0){
            clog<<"[bdy_rt] = ";
            for (int j = 0; j < 6; j++) clog<<node[rt].bdy[j]<<",";
            clog<<"\n";
            clog<<"[bdy_query] = ";
            for (int j = 0; j < 6; j++) clog<<bdy[j]<<",";
            clog<<"\n";
           }
#endif

        // check if they are relavent
        if (!isIntersected(bdy, node[rt].bdy))return;

        // check tag
        if (node[rt].tag == _TAG_MIX)
        {
            for (int chd = 0; chd < _TOT_CHD; chd++)
                query(node[rt].son[chd], from, bdy, graph);
        }
        else if (node[rt].tag == _TAG_EMP)
        {
            double bdy_i[_TOT_BDY];
            retSharedArea(bdy, node[rt].bdy, bdy_i);
#if 0    
           if (from < 0)
           {
            clog<<"[bdy_inter] = ";
            for (int j = 0; j < 6; j++) clog<<bdy_i[j]<<",";
            clog<<"\n";
           }
#endif
            graph->add_bdy_id_id(bdy_i, rt, from);
        }
    }
#endif

    // Q: what if this position is out of boundary? 
    // A: It will return the root.
    int OctoMap::queryPoint(int rt, const double pt[_TOT_DIM])
    {
        //clog << "[OCTOMAP] pt_id = " << rt << endl;
        if (node[rt].son[0] == _NODE_NULL) return rt;
#if 0
        clog << "Pt  : "; clog << pt[0] << ", " << pt[1] << ", " << pt[2] << endl;
        clog << "Box : "; printBlock(node[rt].bdy);
        clog << "son : "; for (auto chd : node[rt].son) clog << chd << ", "; clog << endl;
        clog << "node size : " << node.size() << ", " << graph_node_ptr.size() << endl;
        clog << endl;
#endif

        for (size_t chd = 0; chd < _TOT_CHD; ++chd) 
        {
            if (within(pt, node[node[rt].son[chd]].bdy)) 
                return queryPoint(node[rt].son[chd], pt);
        }
        return rt;
    }


    bool OctoMap::testObstacle(const double pt[_TOT_DIM])
    {
        return node[queryPoint(_NODE_ROOT, pt)].tag != _TAG_EMP;
    }



    void OctoMap::saveAsFile(std::string filename)
    {
        std::ofstream fout(filename.c_str());

        fout << node.size() << "\t" << resolution << std::endl;

        for (std::vector<OctoMap::Node>::iterator it = node.begin();
            it != node.end(); ++it)
        {
            fout << it->id << "\t" << it->tag;

            for (int i = 0; i < _TOT_BDY; i++)
                fout << "\t" << it->bdy[i];

            for (int chd = 0; chd < _TOT_CHD; chd++)
                fout << "\t" << it->son[chd];

            fout << std::endl;
        }
    }

    void OctoMap::loadFromFile(std::string filename)
    {
        std::ifstream fin(filename.c_str());

        int N;
        fin >> N >> resolution;

        node = std::vector<OctoMap::Node>(N);

        for (std::vector<OctoMap::Node>::iterator it = node.begin();
            it!=node.end();it++)
        {
            fin >> it->id >> it->tag;
            
            for (int i = 0; i< _TOT_BDY ; i++)
                fin >> it->bdy[i];

            for (int chd = 0; chd < _TOT_CHD; chd++)
                fin >> it->son[chd];
        }
    }
    
    void OctoMap::retBox(int id,double bdy[_TOT_BDY])
    {
        memcpy(bdy, node[id].bdy, sizeof(node[id].bdy));
    }

#if 0
    void OctoMap::retGraph(VoxelGraph * graph)
    {
        for (auto it = node.begin(); it!=node.end(); ++it)
        {
            if (it->tag != _TAG_EMP) continue;

            double bdy[_TOT_BDY];

            for (int dim = 0; dim < _TOT_DIM; dim++)
            {
                memcpy(bdy, it->bdy, sizeof(bdy));
                bdy[dim << 1] = bdy[dim << 1 | 1];
                bdy[dim << 1 | 1] += eps;

                query(_NODE_ROOT, it->id, bdy, graph);
            }
        }
    }
#endif

    std::vector<double> OctoMap::getPointCloud()
    {
        std::vector<double> pt;

        for (auto it = node.begin(); it != node.end(); ++it)
        {
            if (it->son[0] == _NODE_NULL && it->tag == _TAG_OBS)
            {
                pt.push_back((it->bdy[_BDY_x] + it->bdy[_BDY_X]) * 0.5);
                pt.push_back((it->bdy[_BDY_y] + it->bdy[_BDY_Y]) * 0.5);
                pt.push_back((it->bdy[_BDY_z] + it->bdy[_BDY_Z]) * 0.5);
            }
        }

        //clog << "[TRAJ] Map points already in octomap module." << endl;
        return pt;
    }

    bool OctoMap::testEmpty(const double bdy[_TOT_BDY])
    {
        assert(
                (bdy[_BDY_x] < bdy[_BDY_X]) &&
                (bdy[_BDY_y] < bdy[_BDY_Y]) &&
                (bdy[_BDY_z] < bdy[_BDY_Z])
              );
        bool ret = testEmpty(bdy, _NODE_ROOT);

        dealWithLog();

        return ret;
    }

    bool OctoMap::testEmpty(const double bdy[_TOT_BDY], int rt)
    {
        if (rt == _NODE_NULL) return true;
        if (!isIntersected(bdy, node[rt].bdy)) return true;

        if (node[rt].tag == _TAG_EMP) return true;

        if (testEnclose(bdy, node[rt].bdy) || isAtom(node[rt]))
            return node[rt].tag == _TAG_EMP;

        splitNode(rt);

        for (int chd = 0; chd < _TOT_CHD; chd++)
            if (!testEmpty(bdy, node[rt].son[chd])) 
            {
                return false;
            }

        update(rt);
        return true;
    }

    bool OctoMap::inflateBdy(double bdy[_TOT_BDY], int direction[_TOT_BDY], int inflate_lim)
    {
        int L = 0, R = (node[_NODE_ROOT].bdy[_BDY_X] - node[_NODE_ROOT].bdy[_BDY_x] + eps) 
            / atom[_DIM_x];

        if (inflate_lim >= 0) R = min(R, inflate_lim);

        for (int dim = 0; dim < _TOT_BDY; ++dim)
            if (direction[dim])
                R = min(R, abs(node[_NODE_ROOT].bdy[dim] - bdy[dim] + eps) / atom[dim >> 1]);

        int mid;
       
        while (L < R)
        {
            //clog << "L : " << L << ", Mid : " << mid << ", R : " << R << endl;
            mid = (L + R + 1) >> 1;
            double sub_bdy[_TOT_BDY] = 
            {
                bdy[_BDY_x] + mid * direction[_BDY_x] * atom[_DIM_x],
                bdy[_BDY_X] + mid * direction[_BDY_X] * atom[_DIM_x],
                bdy[_BDY_y] + mid * direction[_BDY_y] * atom[_DIM_y],
                bdy[_BDY_Y] + mid * direction[_BDY_Y] * atom[_DIM_y],
                bdy[_BDY_z] + mid * direction[_BDY_z] * atom[_DIM_z],
                bdy[_BDY_Z] + mid * direction[_BDY_Z] * atom[_DIM_z],
            };
            if (testEmpty(sub_bdy))
                L = mid;
            else
                R = mid - 1;
        }
        //clog << "[Finally] L : " << L << ", R : " << R << endl;

        for (int dim = 0; dim < _TOT_BDY; ++dim)
            bdy[dim] += L * direction[dim] * atom[dim >> 1];

        return testEmpty(bdy);
    }

    pair<Eigen::MatrixXd, Eigen::MatrixXd>
        OctoMap::getPath(const double src[_TOT_DIM], const double dest[_TOT_DIM])
    {
        if (!within(src, node[_NODE_ROOT].bdy) || !within(dest, node[_NODE_ROOT].bdy))
        {
            ROS_WARN("[OCTOMAP] illegal src and dest points.");
            return make_pair(Eigen::MatrixXd(0, 0), Eigen::MatrixXd(0, 0));
        }


        auto src_id = queryPoint(_NODE_ROOT, src);
        auto dest_id = queryPoint(_NODE_ROOT, dest);
        if (node[src_id].tag != _TAG_EMP || node[dest_id].tag != _TAG_EMP) 
            return make_pair(Eigen::MatrixXd(0, 0), Eigen::MatrixXd(0, 0));

        auto p_src_node = graph_node_ptr[src_id];
        auto p_dest_node = graph_node_ptr[dest_id];

        double bdy[_TOT_BDY];

        for (int dim = 0; dim < _TOT_BDY; ++dim) bdy[dim] = src[dim >> 1]; 
        auto p_src_edge = new VoxelGraph::Edge(NULL, p_src_node, bdy);
        p_src_node->nxt[-1] = p_src_edge;

        for (int dim = 0; dim < _TOT_BDY; ++dim) bdy[dim] = dest[dim >> 1];
        auto p_dest_edge = new VoxelGraph::Edge(p_dest_node, NULL, bdy);
        p_dest_node->nxt[-2] = p_dest_edge;
        
        vector<decltype(p_src_edge)> src_edges{p_src_edge};

        auto ret = VoxelGraph::getPathAStar(p_src_edge, p_dest_edge);

        p_src_node->nxt.erase(-1);
        delete p_src_edge;

        p_dest_node->nxt.erase(-2);
        delete p_dest_edge;

        return ret;
    }
    
    pair<Eigen::MatrixXd, Eigen::MatrixXd>
        OctoMap::getPathMultiStart(
                const vector<double> & src, 
                const vector<double> & dest)
    {
        assert(src.size() % _TOT_DIM == 0);
        assert(!src.empty());
        assert((int)dest.size() == _TOT_DIM);
        int n = src.size() / _TOT_DIM;
        auto _EMPTY = make_pair(Eigen::MatrixXd(0, 0), Eigen::MatrixXd(0, 0));
      
        vector<VoxelGraph::Edge *> start_edges(n);
        vector<VoxelGraph::Node *> start_nodes(n);

        for (int idx = 0; idx < n; ++idx)
        {
            if (!within(src.data() + idx * _TOT_DIM, node[_NODE_ROOT].bdy))
            {
                ROS_WARN("[OCTOMAP] illegal start points.");
                return _EMPTY;
            }

            auto src_id = queryPoint(_NODE_ROOT, src.data() + idx * _TOT_DIM);
            if (node[src_id].tag != _TAG_EMP) return _EMPTY;
            double bdy[_TOT_BDY]
            {
                src[idx * _TOT_DIM + _DIM_x], src[idx * _TOT_DIM + _DIM_x],
                src[idx * _TOT_DIM + _DIM_y], src[idx * _TOT_DIM + _DIM_y],
                src[idx * _TOT_DIM + _DIM_z], src[idx * _TOT_DIM + _DIM_z]
            };
            start_nodes[idx] = graph_node_ptr[src_id];
            start_edges[idx] = new VoxelGraph::Edge(start_nodes[idx], NULL, bdy);
            start_nodes[idx]->nxt[-(idx + 1)] = start_edges[idx];
        }
        
        for (int idx = 0; idx < n; ++idx)
            if (!within(dest.data(), node[_NODE_ROOT].bdy))
            {
                ROS_WARN("[OCTOMAP] illegal destination.");
                return _EMPTY;
            }

        auto dest_id = queryPoint(_NODE_ROOT, dest.data());
        if (node[dest_id].tag != _TAG_EMP) return _EMPTY;
        double bdy[_TOT_BDY]
        {
            dest[_DIM_x], dest[_DIM_x],
            dest[_DIM_y], dest[_DIM_y],
            dest[_DIM_z], dest[_DIM_z]
        };
        auto p_dest_node = graph_node_ptr[dest_id];
        auto p_dest_edge = new VoxelGraph::Edge(p_dest_node, NULL, bdy);
        p_dest_node->nxt[-(n + 1)] = p_dest_edge;


        auto ret = VoxelGraph::getPathAStar(start_edges, p_dest_edge);

        for (int idx = 0; idx < n; ++idx)
        {
            delete start_edges[idx];
            start_nodes[idx]->nxt.erase(-(idx + 1));
        }
        delete p_dest_edge;
        p_dest_node->nxt.erase(-(n + 1));
        return ret;
    }


    bool OctoMap::retPathFromTraj(
                pair<Eigen::MatrixXd, Eigen::MatrixXd> & path,
                const Eigen::MatrixXd & coef,
                const double t_start,
                const double t_end,
                double step)
    {
        assert(coef.cols() >= _TOT_DIM);

        vector<int> id_path;
        vector<pair<double, double> > t_path;
        int n = coef.rows();

        auto isWithin = [&] (const vector<double> & pt, int gid)->bool
        {
            return within(pt.data(), node[gid].bdy);
        };

        auto getTrajPoint = [&coef, &n](double t)-> vector<double>
        {
            vector<double> pt {0.0, 0.0, 0.0};
            double T = 1.0;
            for (int i = 0; i < n; ++i, T *= t)
            {
                pt[_DIM_x] += coef(i, _DIM_x) * T;
                pt[_DIM_y] += coef(i, _DIM_y) * T;
                pt[_DIM_z] += coef(i, _DIM_z) * T;
            }
            return pt;
        };

        for (double t = t_start; t <= t_end;)
        {
            auto pt = getTrajPoint(t);
            int gid = queryPoint(_NODE_ROOT, pt.data());


            // in case that pt doest exists in the map
            if (!isWithin(pt, gid)) return false;


            // find a time that is beyond t_end or out of the grid
            double t_next = t;
            while (t_next < t_end && isWithin(getTrajPoint(t_next), gid)) 
                t_next += step;

            // t is beyond t_end
            if (isWithin(getTrajPoint(t_next), gid))
            {
                id_path.push_back(gid);
                t_path.push_back(make_pair(t, t_end));
                break;
            }
            else
            {
                // t_next out of boundary, move to the end of the segment
                double l = t, r = t_next, mid;
                while (l + eps < r)
                {
                    mid = (l + r) * 0.5;
                    if (isWithin(getTrajPoint(mid), gid))
                        l = mid;
                    else
                        r = mid;
                }
                id_path.push_back(gid);
                t_path.push_back(make_pair(t, l));
                t = r + eps;
            }
        }
#if 1
        ROS_INFO("[corridor] T = [%lf, %lf]", t_start, t_end);
        ROS_INFO_STREAM("[corridor] traj : \n" << coef);
        {
            auto x = getTrajPoint(t_start), y = getTrajPoint(t_end);
            ROS_INFO("start point [%lf, %lf, %lf]", x[0], x[1], x[2]);
            ROS_INFO("final point [%lf, %lf, %lf]", y[0], y[1], y[2]);
        }

        for (int idx = 0; idx < id_path.size(); ++idx)
        {
            int id = id_path[idx];
            ROS_INFO("[corridor] id = %d, time = (%lf, %lf), bdy = [%lf,%lf,%lf,%lf,%lf,%lf]",
                    id_path[idx], t_path[idx].first, t_path[idx].second,
                    node[id].bdy[0], node[id].bdy[1], 
                    node[id].bdy[2], node[id].bdy[3],
                    node[id].bdy[4], node[id].bdy[5]);
        }
#endif

        { // return the path, deal with obstacled grids
            int m = id_path.size();
            vector<Eigen::RowVectorXd> grid;
            vector<Eigen::RowVectorXd> face;
            vector<pair<double, double> > time;

            // get the center of the grid
            auto getGirdCenter = [&](int id)->vector<double>
            {
                vector<double> ret 
                {
                    (node[id].bdy[_BDY_x] + node[id].bdy[_BDY_X]) * 0.5,
                    (node[id].bdy[_BDY_y] + node[id].bdy[_BDY_Y]) * 0.5,
                    (node[id].bdy[_BDY_z] + node[id].bdy[_BDY_Z]) * 0.5
                };
                return  ret;
            };


            // assuming that it start with a empty grid;
            grid.push_back(Eigen::Map<Eigen::RowVectorXd>
                    (node[id_path.front()].bdy, _TOT_BDY));
            time.push_back(t_path.front());


            // try to connect two collision-free grid
            auto connectGrids = [&](int i, int j)->void
            {
                // the id of the two grid
                int u = id_path[i], v = id_path[j];
                if (u == v) return;

                // if them are connected directly.
                auto it = graph_node_ptr[u]->nxt.find(v);
                if (it != graph_node_ptr[u]->nxt.end())
                {
                    face.push_back(Eigen::Map<Eigen::RowVectorXd>
                            (it->second->bdy, _TOT_BDY));
                }
                else // otherwise, use A* to find a path that connects them
                {
                    auto pt_start = getGirdCenter(u), pt_goal = getGirdCenter(v);
                    auto local_path = getPath(pt_start.data(), pt_goal.data());

                    Eigen::MatrixXd f = local_path.first.transpose();
                    Eigen::MatrixXd g = local_path.second.transpose();
                    //ROS_WARN_STREAM("[octomap]\n face = \n" << f);
                    //ROS_WARN_STREAM("[octomap]\n grid = \n" << g);

                    // skip the first and last grids
                    for (int idx = 1; idx + 1 < g.rows(); ++idx)
                    {
                        grid.push_back(g.row(idx));
                        time.push_back(make_pair(-1.0, -2.0));
                    }

                    // skip the first and last faces (virtual)
                    for (int idx = 1; idx + 1 < f.rows(); ++idx)
                    {
                        face.push_back(f.row(idx));
                    }
                }

                // push the back node in.
                grid.push_back(Eigen::Map<Eigen::RowVectorXd>
                        (node[v].bdy, _TOT_BDY));
                time.push_back(t_path[j]);
            };

            // iterate a free grid
            for (int i = 0; i < m;)
            {
                if (node[id_path[i]].tag == _TAG_EMP)
                {
                    int j = i + 1;
                    for (; j < m; ++j)
                    {
                        if (node[id_path[j]].tag != _TAG_EMP) 
                            continue;
                        else
                        {
                            connectGrids(i, j);
                            break;
                        }
                    }
                    i = j;
                }
                else
                    i = i + 1;
            }

            Eigen::MatrixXd G(grid.size(), _TOT_BDY + 2), E(face.size(), _TOT_BDY); 

            for (int i = 0; i < grid.size(); ++i)
            {
                G.row(i) << grid[i], time[i].first, time[i].second;
            }

            for (int i = 0; i < face.size(); ++i)
            {
                E.row(i) << face[i];
            }

            path = make_pair(G, E);
            return true;
        }
    }

#if 0
    bool OctoMap::retPathFromTraj(
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> & path,
        const Eigen::VectorXd & init_pos, 
        const Eigen::MatrixXd & coef, 
        const double t_start, 
        const double t_end,
        double step)
    {
        assert(init_pos.size() >= _TOT_DIM);
        assert(coef.cols() >= _TOT_DIM);
        //ROS_WARN("[octomap] search corridor with given start.");
        vector<int> id_path;
        vector<pair<double, double> > t_path;
        int n = coef.rows();

        auto isWithin = [&] (const vector<double> & pt, int gid)->bool
        {
            return within(pt.data(), node[gid].bdy);
        };

        auto getTrajPoint = [&coef, &n](double t)-> vector<double>
        {
            vector<double> pt {0.0, 0.0, 0.0};
            double T = 1.0;
            for (int i = 0; i < n; ++i, T *= t)
            {
                pt[_DIM_x] += coef(i, _DIM_x) * T;
                pt[_DIM_y] += coef(i, _DIM_y) * T;
                pt[_DIM_z] += coef(i, _DIM_z) * T;
            }
            return pt;
        };

        for (double t = t_start; t <= t_end;)
        {
            auto pt = getTrajPoint(t);
            int gid = queryPoint(_NODE_ROOT, pt.data());


            // in case that pt doest exists in the map
            if (!isWithin(pt, gid)) return false;


            // find a time that is beyond t_end or out of the grid
            double t_next = t;
            while (t_next < t_end && isWithin(getTrajPoint(t_next), gid)) 
                t_next += step;

            // t is beyond t_end
            if (isWithin(getTrajPoint(t_next), gid))
            {
                id_path.push_back(gid);
                t_path.push_back(make_pair(t, t_end));
                break;
            }

            // t_next out of boundary, move to the end of the segment
            double l = t - step, r = t_next, mid;
            while (l + eps < r)
            {
                mid = (l + r) * 0.5;
                if (isWithin(getTrajPoint(mid), gid))
                    l = mid;
                else
                    r = mid;
            }
            id_path.push_back(gid);
            t_path.push_back(make_pair(t, l));
            t = r;
        }
#if 1
        ROS_INFO("[corridor] T = [%lf, %lf]", t_start, t_end);
        ROS_INFO_STREAM("[corridor] traj : \n" << coef);
        {
            auto x = getTrajPoint(t_start), y = getTrajPoint(t_end);
            ROS_INFO("start point [%lf, %lf, %lf]", x[0], x[1], x[2]);
            ROS_INFO("final point [%lf, %lf, %lf]", y[0], y[1], y[2]);
        }

        for (int idx = 0; idx < id_path.size(); ++idx)
        {
            int id = id_path[idx];
            ROS_INFO("[corridor] [%d, %d], (%lf, %lf), bdy = [%lf,%lf,%lf,%lf,%lf,%lf]",
                    id_path[idx], node[id_path[idx]].tag,
                    t_path[idx].first, t_path[idx].second,
                    node[id].bdy[0], node[id].bdy[1], 
                    node[id].bdy[2], node[id].bdy[3],
                    node[id].bdy[4], node[id].bdy[5]);
        }
#endif

        { // return the path, deal with obstacled grids
            int m = id_path.size();
            vector<Eigen::RowVectorXd> grid;
            vector<Eigen::RowVectorXd> face;
            vector<pair<double, double> > time;
            
            // get the center of the grid
            auto getGirdCenter = [&](int id)->vector<double>
            {
                vector<double> ret 
                {
                    (node[id].bdy[_BDY_x] + node[id].bdy[_BDY_X]) * 0.5,
                    (node[id].bdy[_BDY_y] + node[id].bdy[_BDY_Y]) * 0.5,
                    (node[id].bdy[_BDY_z] + node[id].bdy[_BDY_Z]) * 0.5
                };
                return  ret;
            };
            
            {
                pair<Eigen::MatrixXd, Eigen::MatrixXd> local_path;
                for (int i = 0; i < m; ++i)
                    if (node[id_path[i]].tag == _TAG_EMP)
                    {
                        local_path = 
                            getPathMultiStart(vector<double>(init_pos.data(), init_pos.data() + _TOT_DIM), getGirdCenter(id_path[i]));
                        break;
                    }
                    
                if (local_path.second.rows() == 0) 
                {
                    ROS_WARN("[OCTOMAP] no path form current position to corridor");
                    return false;
                }
                    
                    
                ROS_WARN_STREAM("[face]\n" << local_path.first.transpose() << "\n[grid]\n" << local_path.second.transpose());

                for (int i = 1; i + 1 < local_path.first.cols(); ++i)
                    face.push_back(local_path.first.col(i).transpose());

                for (int i = 0; i < local_path.second.cols(); ++i)
                {
                    grid.push_back(local_path.second.col(i).transpose());
                    time.push_back(pair<double,double>(-1.0, -2.0));
                }
            }

            // try to connect two collision-free grid
            auto connectGrids = [&](int i, int j)->bool
            {
                // the id of the two grid
                int u = id_path[i], v = id_path[j];
                if (u == v) return true;

                // if them are connected directly.
                auto it = graph_node_ptr[u]->nxt.find(v);
                if (it != graph_node_ptr[u]->nxt.end())
                {
                    face.push_back(Eigen::Map<Eigen::RowVectorXd>
                            (it->second->bdy, _TOT_BDY));
                }
                else // otherwise, use A* to find a path that connects them
                {
                    auto pt_start = getGirdCenter(u), pt_goal = getGirdCenter(v);
                    //auto local_path = getPath(pt_start.data(), pt_goal.data());
                    auto local_path = getPathMultiStart(pt_start, pt_goal);

                    Eigen::MatrixXd f = local_path.first.transpose();
                    Eigen::MatrixXd g = local_path.second.transpose();
                    
                    if (g.rows() == 0) return false;
                    //ROS_WARN_STREAM("[octomap]\n face = \n" << f);
                    //ROS_WARN_STREAM("[octomap]\n grid = \n" << g);

                    // skip the first and last grids
                    for (int idx = 1; idx + 1 < g.rows(); ++idx)
                    {
                        grid.push_back(g.row(idx));
                        time.push_back(make_pair(-1.0, -2.0));
                    }

                    // skip the first and last faces (virtual)
                    for (int idx = 1; idx + 1 < f.rows(); ++idx)
                    {
                        face.push_back(f.row(idx));
                    }
                }

                // push the back node in.
                grid.push_back(Eigen::Map<Eigen::RowVectorXd>
                        (node[v].bdy, _TOT_BDY));
                time.push_back(t_path[j]);
                return true;
            };

            // iterate a free grid
            for (int i = 0; i < m;)
            {
                if (node[id_path[i]].tag == _TAG_EMP)
                {
                    int j = i + 1;
                    for (; j < m; ++j)
                    {
                        if (node[id_path[j]].tag != _TAG_EMP) 
                            continue;
                        else
                        {
                            if (!connectGrids(i, j)) return false;
                            break;
                        }
                    }
                    i = j;
                }
                else
                    i = i + 1;
            }

            Eigen::MatrixXd G(grid.size(), _TOT_BDY + 2), E(face.size(), _TOT_BDY); 
#if 0
            ROS_WARN("[grid]");
            for (auto & r : grid) ROS_WARN_STREAM(":" << r);
            ROS_WARN("[face]");
            for (auto & r : face) ROS_WARN_STREAM(":" << r);
#endif
            for (int i = 0; i < grid.size(); ++i)
            {
                G.row(i) << grid[i], time[i].first, time[i].second;
            }

            for (int i = 0; i < face.size(); ++i)
            {
                E.row(i) << face[i];
            }

            path = make_pair(G, E);
            return true;
        }



    }
#else
    bool OctoMap::retPathFromTraj(
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> & path,
        const Eigen::VectorXd & init_pos, 
        const Eigen::MatrixXd & coef, 
        const double t_start, 
        const double t_end,
        double & allowed_t_end,
        double step)
    {
        assert(init_pos.size() >= _TOT_DIM);
        assert(coef.cols() >= _TOT_DIM);
        //ROS_WARN("[octomap] search corridor with given start.");
        vector<int> id_path;
        vector<pair<double, double> > t_path;
        int n = coef.rows();

        auto isWithin = [&] (const vector<double> & pt, int gid)->bool
        {
            return within(pt.data(), node[gid].bdy);
        };

        auto getTrajPoint = [&coef, &n](double t)-> vector<double>
        {
            vector<double> pt {0.0, 0.0, 0.0};
            double T = 1.0;
            for (int i = 0; i < n; ++i, T *= t)
            {
                pt[_DIM_x] += coef(i, _DIM_x) * T;
                pt[_DIM_y] += coef(i, _DIM_y) * T;
                pt[_DIM_z] += coef(i, _DIM_z) * T;
            }
            return pt;
        };

        for (double t = t_start; t <= t_end;)
        {
            auto pt = getTrajPoint(t);
            int gid = queryPoint(_NODE_ROOT, pt.data());


            // in case that pt doest exists in the map
            if (!isWithin(pt, gid)) return false;


            // find a time that is beyond t_end or out of the grid
            double t_next = t;
            while (t_next < t_end && isWithin(getTrajPoint(t_next), gid)) 
                t_next += step;

            // t is beyond t_end
            if (isWithin(getTrajPoint(t_next), gid))
            {
                id_path.push_back(gid);
                t_path.push_back(make_pair(t, t_end));
                break;
            }

            // t_next out of boundary, move to the end of the segment
            double l = t, r = t_next, mid;
            while (l + eps < r)
            {
                mid = (l + r) * 0.5;
                if (isWithin(getTrajPoint(mid), gid))
                    l = mid;
                else
                    r = mid;
            }
            id_path.push_back(gid);
            t_path.push_back(make_pair(t, l));
            t = r;
        }
#if 0
        ROS_INFO("[corridor] T = [%lf, %lf]", t_start, t_end);
        ROS_INFO_STREAM("[corridor] traj : \n" << coef);
        {
            auto x = getTrajPoint(t_start), y = getTrajPoint(t_end);
            ROS_INFO("start point [%lf, %lf, %lf]", x[0], x[1], x[2]);
            ROS_INFO("final point [%lf, %lf, %lf]", y[0], y[1], y[2]);
        }

        for (int idx = 0; idx < (int)id_path.size(); ++idx)
        {
            int id = id_path[idx];
            ROS_INFO("[corridor] [%d, %d], (%lf, %lf), bdy = [%lf,%lf,%lf,%lf,%lf,%lf]",
                    id_path[idx], node[id_path[idx]].tag,
                    t_path[idx].first, t_path[idx].second,
                    node[id].bdy[0], node[id].bdy[1], 
                    node[id].bdy[2], node[id].bdy[3],
                    node[id].bdy[4], node[id].bdy[5]);
        }
#endif

        { // return the path, deal with obstacled grids
            vector<Eigen::RowVectorXd> grid;
            vector<Eigen::RowVectorXd> face;
            
            // get the center of the grid
            auto getGridCenter = [&](int id)->vector<double>
            {
                vector<double> ret 
                {
                    (node[id].bdy[_BDY_x] + node[id].bdy[_BDY_X]) * 0.5,
                    (node[id].bdy[_BDY_y] + node[id].bdy[_BDY_Y]) * 0.5,
                    (node[id].bdy[_BDY_z] + node[id].bdy[_BDY_Z]) * 0.5
                };
                return  ret;
            };

            vector<int>::iterator _tail;
            auto pathToCorridor = [&](
                    const vector<double> && pos,
                    const vector<int>::iterator _beg, 
                    const vector<int>::iterator _end,
                    bool is_first  = false)
                ->bool
            {
                // obstain the start positions
                vector<double> dst;
                for (auto it = _beg; it != _end; ++it)
                {
                    if (node[*it].tag != _TAG_EMP) continue;
                    auto ctr = getGridCenter(*it);
                    dst.insert(dst.end(), ctr.begin(), ctr.end());
                }

                if (dst.empty()) return false;

                // get the corridor
                auto _path = getPathMultiStart(dst, pos);
                auto f = _path.first.transpose();
                auto g = _path.second.transpose();

                // whether legal
                //ROS_WARN_STREAM("number of rows:" << g.rows());
                //ROS_WARN_STREAM("face:" << endl << f << endl);
                //ROS_WARN_STREAM("grid:" << endl << g << endl);
                if (g.rows() == 0) return false;

                // install faces and corridor
                for (int i = f.rows() - 2; i > 0; --i) face.push_back(f.row(i));

                // if its the first segment, store the first grid
                if (is_first) grid.push_back(g.row(g.rows() - 1));
                for (int i = g.rows() - 2; i >= 0; --i) grid.push_back(g.row(i));

                double pt [_TOT_DIM] {f(0, _BDY_x), f(0, _BDY_y), f(0, _BDY_z)};

                // check the actual destination, and return 
                for (auto it = _beg; it != _end; ++it)
                    if (within(pt, node[*it].bdy)) 
                    {
                        _tail = it;
                        return true;
                    }

                //ROS_WARN("no tail");
                // illegal destination
                return false;
            };
            
            // ROS_WARN("Let's dance!" );
            if (!pathToCorridor(
                    vector<double>(init_pos.data(), init_pos.data() + _TOT_DIM),
                    id_path.begin(), id_path.end(), true)) 
                return false;
            //ROS_WARN("First segment ready, tail = %d", (int)(_tail - begin(id_path)));
            //for (auto & row: face) ROS_WARN_STREAM("face : " << row);
            //for (auto & row: grid) ROS_WARN_STREAM("grid : " << row);

            allowed_t_end =  t_end;
            while (next(_tail) != end(id_path))
            {
                for (auto _next = next(_tail); _next != end(id_path); _next = next(_next))
                { 
                    auto p_edge = graph_node_ptr[*_tail]->nxt.find(*_next);
                    if (p_edge == end(graph_node_ptr[*_tail]->nxt)) break;
                    face.push_back(Eigen::RowVectorXd::Map(p_edge->second->bdy, _TOT_BDY));
                    grid.push_back(Eigen::RowVectorXd::Map(node[*_next].bdy, _TOT_BDY));
                    _tail = _next;
                }
                
                if (next(_tail) != end(id_path))
                    allowed_t_end = t_path[_tail - id_path.begin()].second;
                else
                    allowed_t_end =  t_end;
                    
                if (next(_tail) != end(id_path) && !pathToCorridor(
                            getGridCenter(*_tail),
                            next(_tail), end(id_path)))
                    break;
            }

            //ROS_WARN("All segment ready, tail = %d", (int)(_tail - begin(id_path)));
            Eigen::MatrixXd F(face.size(), _TOT_BDY), G(grid.size(), _TOT_BDY);
            for (size_t i = 0; i < face.size(); ++i) F.row(i) << face[i];
            for (size_t i = 0; i < grid.size(); ++i) G.row(i) << grid[i];
            path = make_pair(G, F);
        }
        return true;
    }
#endif
}
