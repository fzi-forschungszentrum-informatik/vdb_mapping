
// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2021 FZI Forschungszentrum Informatik
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
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lennart Puck puck@fzi.de
 * \date    2021-04-29
 *
 */
//----------------------------------------------------------------------

#ifndef VDB_MAPPING_DATA_NODE_H_INCLUDED
#define VDB_MAPPING_DATA_NODE_H_INCLUDED


// TODO: maybe implement more operators (needed for openvdb), compiles for now
class DataNode
{
public:

  //! Construct an empty DataNode
  DataNode() {m_data = 0.0f;}

  DataNode(float data){m_data = data;}

  //! Deconstructor
  virtual ~DataNode() {}

  //void setData(const float data);

  //float getData() const;

  void updateNode(const float data) {
    m_data = data;
  }

  float getData() const
  {
    return m_data;
  }



  ///////////////////////////////////////
  // Operator overloads
  //////////////////////////////////////

  DataNode operator-() const
  {
    // TODO: properly implement DataNode operator-
    return DataNode();
  }

  bool operator==(const DataNode& other) const
  {
    // TODO: properly implement DataNode operator==
    return true;
  }

  DataNode operator-(const DataNode& other) const
  {
    // TODO: properly implement DataNode operator-
    return other;
  }

  DataNode operator+(const DataNode& other) const
  {
    // TODO: properly implement DataNode operator+
    return other;
  }

  bool operator>(const DataNode& other) const
  {
    // TODO: properly implement DataNode operator>
    return true;
  }

  bool operator<(const DataNode& other) const
  {
    // TODO: properly implement DataNode operator<
    return true;
  }

protected:
  float m_data; 

};

inline DataNode Abs(const DataNode& node)
{
  // TODO: properly implement DataNode Abs
  return node;
}

inline std::ostream& operator<<(std::ostream& ostr, const DataNode& node)
{
  // TODO: properly implement DataNode operator<<
  ostr << "Hello, I'm a DataNode";
  return ostr;
}

//TODO understand what these should be doing and fill them correctly
inline DataNode operator*(float scalar, const DataNode &v) { return scalar; }
inline DataNode operator*(const DataNode &u, const DataNode &v) { return 1; }


#endif

