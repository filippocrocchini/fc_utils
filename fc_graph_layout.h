/*
Author:
	- Filippo Crocchini

Copyright:
	This library is available under the MIT license, see end of the file.

Usage:
    // Do this in only one file

    #define FC_GRAPH_LAYOUT_IMPLEMENTATION
    #include "fc_graph_layout.h"

    // Just include as usual in the others
    #include "fc_graph_layout.h"

*/

#ifndef FC_GRAPH_LAYOUT
#define FC_GRAPH_LAYOUT

struct fc_v2f
{
    float x;
    float y;
};

struct fc_node
{
    fc_v2f position;
};

struct fc_edge
{
    int first;
    int second;
};

struct fc_graph
{
    fc_node* nodes;
    int node_count;

    fc_edge* edges;
    int edge_count;
};

struct fc_layout_info
{
    float min_energy = 1.f;

    float repulsive_force_scale = 0.6f;
    float optimal_distance      = 2000;

    float step = 100;
    float t    = 0.9f;
};

void layout_graph(fc_graph graph, fc_layout_info layout_info);

#ifdef FC_GRAPH_LAYOUT_IMPLEMENTATION

#include <math.h>  // sqrtf
#include <float.h> // FLT_EPSILON

static fc_v2f fc_v2f_subtract(fc_v2f a, fc_v2f b)
{
    a.x -= b.x;
    a.y -= b.y;
    return a;
}

static fc_v2f fc_v2f_add(fc_v2f a, fc_v2f b)
{
    a.x += b.x;
    a.y += b.y;
    return a;
}

static fc_v2f fc_v2f_multiply(fc_v2f a, float b)
{
    a.x *= b;
    a.y *= b;
    return a;
}

static float fc_v2f_length_sq(fc_v2f a)
{
    return a.x*a.x + a.y*a.y;
}

static float fc_v2f_length(fc_v2f a)
{
    return sqrtf(fc_v2f_length_sq(a));
}

static fc_v2f fc_v2f_normalize(fc_v2f a)
{
    auto len = fc_v2f_length(a);

    if(len < FLT_EPSILON)
        return fc_v2f{};

    return fc_v2f_multiply(a, 1.f / len);
}

static fc_v2f attractive_force(fc_node* a, fc_node* b, float optimal_distance)
{
    fc_v2f diff = fc_v2f_subtract(b->position, a->position);
    return fc_v2f_multiply(diff, fc_v2f_length(diff) / optimal_distance);
}

static fc_v2f repulsive_force(fc_node* a, fc_node* b, float repulsive_force_scale, float optimal_distance)
{
    fc_v2f diff = fc_v2f_subtract(b->position, a->position);
    float dist = fc_v2f_length(diff);
    
    if(dist < FLT_EPSILON)
        return fc_v2f{};

    return fc_v2f_multiply(diff, -repulsive_force_scale * optimal_distance / (dist*dist*dist));
}

static float compute_adaptive_step(int* progress, float t, float step, float last_energy, float energy)
{
    if(energy < last_energy){
        *progress += 1;
        if(*progress >= 5){
            *progress = 0;
            return step / t;
        }
    } else {
        *progress = 0;
        return t * step;
    }
    return step;
}

void layout_graph(fc_graph graph, fc_layout_info layout_info)
{
    float step       = layout_info.step;
    float min_energy = layout_info.min_energy;

    float t = layout_info.t;
    float repulsive_force_scale = layout_info.repulsive_force_scale;
    float optimal_distance = layout_info.optimal_distance;

    float energy = 0;
    int progress = 0;

    while(true)
    {
        float last_energy = energy;
        energy = 0;

        for(int i = 0; i < graph.node_count; i++)
        {
            auto node = graph.nodes + i;

            fc_v2f force = {};
            for(int j = 0; j < graph.edge_count; j++)
            {
                auto edge = graph.edges[j];

                if(edge.first == edge.second) continue;

                int other = -1;
                if(edge.first == i){
                    other = edge.second;
                } else if(edge.second == i){
                    other = edge.first;
                }

                if(other >= 0)
                {
                    force = fc_v2f_add(force, attractive_force(node, graph.nodes + other, optimal_distance));
                }
            }

            for(int j = 0; j < graph.node_count; j++)
            {
                auto other_node = graph.nodes + j;

                if(i == j) continue;

                force = fc_v2f_add(force, repulsive_force(node, other_node, repulsive_force_scale, optimal_distance));
            }

            fc_v2f dp = fc_v2f_multiply(fc_v2f_normalize(force), step);
            node->position = fc_v2f_add(node->position, dp);
            energy += fc_v2f_length_sq(force);
        }

        step = compute_adaptive_step(&progress, t, step, last_energy, energy);

        if(energy < min_energy) break;
    }
}

#endif // FC_GRAPH_LAYOUT_IMPLEMENTATION

#endif // FC_GRAPH_LAYOUT

/*
	Copyright (c) 2022 Filippo Crocchini

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/