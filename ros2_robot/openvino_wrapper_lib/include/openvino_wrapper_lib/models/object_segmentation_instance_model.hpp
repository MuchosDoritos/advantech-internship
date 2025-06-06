// Copyright (c) 2023 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef OPENVINO_WRAPPER_LIB__MODELS__OBJECT_SEGMENTATION_INSTANCE_MODEL_HPP_
#define OPENVINO_WRAPPER_LIB__MODELS__OBJECT_SEGMENTATION_INSTANCE_MODEL_HPP_
#include <string>
#include <openvino/openvino.hpp>
#include "openvino_wrapper_lib/models/base_model.hpp"

namespace openvino_wrapper_lib
{
  class ObjectSegmentationInstanceResult;
}

namespace Models
{

/**
 * @class ObjectSegmentationInstanceModel
 * @brief This class generates the object segmentation model.
 */
class ObjectSegmentationInstanceModel : public BaseModel
{
  using Result = openvino_wrapper_lib::ObjectSegmentationInstanceResult;
public:
  ObjectSegmentationInstanceModel(const std::string& label_loc, const std::string & model_loc, int batch_size = 1);

  virtual bool fetchResults(
    const std::shared_ptr<Engines::Engine> & engine,
    std::vector<openvino_wrapper_lib::ObjectSegmentationInstanceResult> & results,
    const float & confidence_thresh = 0.3,
    const bool & enable_roi_constraint = false);

  bool enqueue(const std::shared_ptr<Engines::Engine> & ,const cv::Mat &,
    const cv::Rect & ) override;

  /**
   * @brief Get the name of this segmentation model.
   * @return Name of the model.
   */
  const std::string getModelCategory() const override;
  virtual bool updateLayerProperty(std::shared_ptr<ov::Model>&) override;

};
}  // namespace Models
#endif  // OPENVINO_WRAPPER_LIB__MODELS__OBJECT_SEGMENTATION_INSTANCE_MODEL_HPP_
