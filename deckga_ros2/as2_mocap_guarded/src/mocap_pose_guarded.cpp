// Copyright 2026 Mitchell Solomon
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the conditions in the BSD-3-Clause
// licence reproduced in include/as2_mocap_guarded/mocap_pose_guarded.hpp are met.

/**
 * @file mocap_pose_guarded.cpp
 *
 * The only translation unit of the plugin library. Kept deliberately tiny: the
 * class lives inline in the header (matching upstream mocap_pose.hpp), and this
 * file exists so the shared object contains exactly one thing -- the plugin --
 * which is what class_loader wants (see its "A metaobject exists for desired
 * class, but has no owner" warning about libraries containing more than plugins).
 *
 * The library name here MUST stay `mocap_pose_guarded`, because pluginlib
 * resolves the class through `<library path="mocap_pose_guarded">` in plugins.xml
 * and the stock as2_state_estimator node builds its lookup name as
 * `<plugin_name parameter> + "::Plugin"`.
 */

#include "as2_mocap_guarded/mocap_pose_guarded.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mocap_pose_guarded::Plugin,
  as2_state_estimator_plugin_base::StateEstimatorBase)
