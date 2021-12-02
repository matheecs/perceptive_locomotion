/*
 * Stair.hpp
 *
 *  Created on: 2017年10月6日
 *      Author: zxm
 */

#ifndef STAIR_HPP_
#define STAIR_HPP_

#include "../common.h"

namespace stair_perception
{
    class Line
    {
    public:
        Line() : h(0), d(0) {};

        pcl::ModelCoefficients coeff;
        float h, d;
    };

    class Step
    {
    public:
        Step() : height(0), depth(0), count(0),
                 good_h(false), good_d(false),
                 plane_h(nullptr), plane_v(nullptr) {}

        Plane *plane_v, *plane_h;
        Line line;
        int count;
        double height, depth;
        bool good_h, good_d;
    };

    class ConcaveLine
    {
    public:
        Plane *plane_v, *plane_h;
        Line line;
    };

    enum PartType
    {
        concaveline_node,
        step_node,
        start_or_end_node
    };

    class Node
    {
    public:
        Node() : pcurrent_type(PartType::start_or_end_node), pnext_type(PartType::start_or_end_node), pnext(nullptr) {}

        PartType pcurrent_type;

        Step step;
        ConcaveLine concave_line;

        PartType pnext_type;
        std::shared_ptr<Node> pnext;
    };


    class Stair
    {
    public:
        Stair() : step_count(0)
        {
            phead = std::make_shared<Node>();
            pend = std::make_shared<Node>();

            point_current = phead;
        }

        void pushBack(const std::shared_ptr<Node> &pnode)
        {
            point_current->pnext_type = pnode->pcurrent_type;
            point_current->pnext = pnode;
            point_current = pnode;
        }

        void pushBackEnd()
        {
            point_current->pnext_type = start_or_end_node;
            point_current->pnext = pend;
            point_current = pend;
        }

        void reset()
        {
            point_current = phead;
        }

        // read next node in the link list, if exceed to the end, return false
        bool readNext(std::shared_ptr<Node> &next_node)
        {
            if (point_current->pnext == pend)
                return false;

            // move to next node
            point_current = point_current->pnext;

            // copy the node to next_node
            next_node = point_current;

            return true;
        }

        std::shared_ptr<Node> getHead()
        {
            return phead;
        }

        std::shared_ptr<Node> getCurPoint()
        {
            return point_current;
        }

    private:
        std::shared_ptr<Node> phead, pend;
        std::shared_ptr<Node> point_current;
        int step_count;
    };
}


#endif /* STAIR_HPP_ */
