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

    This library partially implements Yifan Hu's graph layout algorithm described in
    "Efficient, High-Quality Force-Directed Graph Drawing", Yifan Hu, 2006
*/

#ifndef FC_GRAPH_LAYOUT
#define FC_GRAPH_LAYOUT

typedef struct
{
    float x;
    float y;
} fc_v2f;

typedef struct
{
    fc_v2f position;
} fc_node;

typedef struct
{
    int first;
    int second;

    float weight;
} fc_edge;

#ifndef __cplusplus
const fc_edge fc_edge_default = fc_edge
{
    .weight = 1.f,
};
#else 
constexpr fc_edge fc_edge_default = {
    0, //first
    0, //second
    1.f, //weight
};
#endif

typedef struct
{
    fc_node* nodes;
    int node_count;

    fc_edge* edges;
    int edge_count;
} fc_graph;

typedef struct
{
    float repulsive_force_scale; 
    float optimal_distance;

    float initial_step_length; // The algorithm uses an adaptive step size, so this is just the starting value.

    int iteration_cap;
    float min_movement; // This is the smalles movement after which we will consider the current configuration to be good enough.

    float central_force_scale;

    float step_multiplier;
} fc_layout_info;

#ifndef __cplusplus
const fc_layout_info fc_layout_info_default = {
    .repulsive_force_scale = 0.6f,
    .optimal_distance      = 16.f,
    .initial_step_length   = 100,
    .iteration_cap         = INT_MAX,
    .min_movement          = 1,
    .central_force_scale   = 0.f,
    .step_multiplier       = 0.9f,
};
#else 
constexpr fc_layout_info fc_layout_info_default = {
    0.6f, // repulsive_force_scale
    16.f, // optimal_distance
    100, // initial_step_length
    INT_MAX, // iteration_cap
    1, // min_movement
    0.f,
    0.9f
};
#endif

typedef struct
{
    float      step;
    float    energy;
    int    progress;
    float biggest_movement_in_iteration;
} fc_dynamic_layout_state;

void fc_begin_dynamic_layout(fc_dynamic_layout_state* state);
void fc_compute_dynamic_step(fc_dynamic_layout_state* state, fc_graph graph, fc_layout_info layout_info);

void fc_layout_graph(fc_graph graph, fc_layout_info layout_info);

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

static fc_v2f fc_attractive_force(fc_v2f p1, fc_v2f p2, float scale, float optimal_distance)
{
    fc_v2f diff = fc_v2f_subtract(p2, p1);
    return fc_v2f_multiply(diff, scale * fc_v2f_length(diff) / optimal_distance);
}

static fc_v2f fc_repulsive_force(fc_v2f p1, fc_v2f p2, float scale, float optimal_distance)
{
    fc_v2f diff = fc_v2f_subtract(p2, p1);
    float dist = fc_v2f_length(diff);

    if(dist < FLT_EPSILON)
        return fc_v2f{};

    return fc_v2f_multiply(diff, - scale * optimal_distance / (dist*dist*dist));
}

static float fc_compute_adaptive_step(int* progress, float t, float step, float last_energy, float energy)
{
    if(energy < last_energy){
        *progress += 1;
        if(*progress >= 5){
            *progress = 0;
            return step / t;
        }
    } else {
        *progress = 0;
        return step * t;
    }
    return step;
}

void fc_begin_dynamic_layout(fc_dynamic_layout_state* state, fc_layout_info layout_info)
{
    state->step      = layout_info.initial_step_length;
    state->energy    = INFINITY;
    state->progress  = 0;
}

void fc_compute_dynamic_step(fc_dynamic_layout_state* state, fc_graph graph, fc_layout_info layout_info)
{
    float optimal_distance      = (layout_info.optimal_distance * layout_info.optimal_distance * layout_info.optimal_distance * layout_info.optimal_distance) / layout_info.repulsive_force_scale;
    float repulsive_force_scale = layout_info.repulsive_force_scale;
    float last_energy           = state->energy;

    state->energy = 0;
    state->biggest_movement_in_iteration = 0;

    for (int i = 0; i < graph.node_count; i++)
    {
        auto node = graph.nodes + i;

        fc_v2f force = {};
        for (int j = 0; j < graph.edge_count; j++)
        {
            auto edge = graph.edges[j];

            if (edge.first == edge.second) continue;

            int other = -1;
            if (edge.first == i)
            {
                other = edge.second;
            }
            else if (edge.second == i) {
                other = edge.first;
            }

            if (other >= 0)
            {
                force = fc_v2f_add(force, fc_attractive_force(node->position, (graph.nodes + other)->position, edge.weight, optimal_distance));
            }
        }

        for (int j = 0; j < graph.node_count; j++)
        {
            auto other_node = graph.nodes + j;

            if (i == j) continue;

            force = fc_v2f_add(force, fc_repulsive_force(node->position, other_node->position, repulsive_force_scale, optimal_distance));
        }

        force = fc_v2f_add(force, fc_attractive_force(node->position, fc_v2f{0, 0}, layout_info.central_force_scale, optimal_distance));

        fc_v2f dp = fc_v2f_multiply(fc_v2f_normalize(force), state->step);
        node->position = fc_v2f_add(node->position, dp);
        state->energy += fc_v2f_length_sq(force);

        float dp_length = fc_v2f_length(dp);
        if (state->biggest_movement_in_iteration < dp_length)
        {
            state->biggest_movement_in_iteration = dp_length;
        }
    }

    state->step = fc_compute_adaptive_step(&state->progress, layout_info.step_multiplier, state->step, last_energy, state->energy);
}

void fc_layout_graph(fc_graph graph, fc_layout_info layout_info)
{
    fc_dynamic_layout_state state = {};

    fc_begin_dynamic_layout(&state, layout_info);

    int iteration = 0;

    while (iteration < layout_info.iteration_cap)
    {
        iteration += 1;
        fc_compute_dynamic_step(&state, graph, layout_info);

        if (state.biggest_movement_in_iteration < layout_info.min_movement) break;
    }
}

#endif // FC_GRAPH_LAYOUT_IMPLEMENTATION

#endif // FC_GRAPH_LAYOUT

/*
    Copyright (c) 2023 Filippo Crocchini

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
