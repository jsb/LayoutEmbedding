#pragma once

#include <polymesh/pm.hh>

bool adjacent(const pm::vertex_handle& _v0, const pm::vertex_handle& _v1);

bool incident(const pm::vertex_handle& _v, const pm::edge_handle& _e);
bool incident(const pm::edge_handle& _e, const pm::vertex_handle& _v);
