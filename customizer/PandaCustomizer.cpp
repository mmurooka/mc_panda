#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <cmath>
#include <cstring>

#ifdef CNOID_BODY_CUSTOMIZER
#  include <cnoid/BodyCustomizerInterface>
#else
#  include <BodyCustomizerInterface.h>
#endif

#include <iostream>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#  define DLL_EXPORT __declspec(dllexport)
#else
#  define DLL_EXPORT
#endif /* Windows */

#if defined(HRPMODEL_VERSION_MAJOR) && defined(HRPMODEL_VERSION_MINOR)
#  if HRPMODEL_VERSION_MAJOR >= 3 && HRPMODEL_VERSION_MINOR >= 1
#    include <hrpUtil/Eigen3d.h>
#    define NS_HRPMODEL hrp
#  endif
#endif

#ifdef CNOID_BODY_CUSTOMIZER
#  define NS_HRPMODEL cnoid
typedef cnoid::Matrix3 Matrix33;
#endif

using namespace std;
using namespace boost;
using namespace NS_HRPMODEL;

static const bool debugMode = false;

static BodyInterface * bodyInterface = 0;

static BodyCustomizerInterface bodyCustomizerInterface;

enum ModelType
{
  Panda = 1
};

struct JointValSet
{
  double * valuePtr;
  double * velocityPtr;
  double * torqueForcePtr;
};

struct PandaCustomizer
{
  BodyHandle bodyHandle;

  int modelType;

  bool hasVirtualBushJoints;
  JointValSet jointValSets[6];
  double springT;
  double dampingT;
  double springR;
  double dampingR;
};

static const char ** getTargetModelNames()
{
  static const char * names[] = {"Panda", 0};

  return names;
}

static void getVirtualbushJoints(PandaCustomizer * customizer, BodyHandle body)
{
  customizer->hasVirtualBushJoints = true;

  int bushIndices[6];

  bushIndices[0] = bodyInterface->getLinkIndexFromName(body, "BUSH_X");
  bushIndices[1] = bodyInterface->getLinkIndexFromName(body, "BUSH_Y");
  bushIndices[2] = bodyInterface->getLinkIndexFromName(body, "BUSH_Z");
  bushIndices[3] = bodyInterface->getLinkIndexFromName(body, "BUSH_ROLL");
  bushIndices[4] = bodyInterface->getLinkIndexFromName(body, "BUSH_PITCH");
  bushIndices[5] = bodyInterface->getLinkIndexFromName(body, "BUSH_YAW");

  for(int j = 0; j < 6; ++j)
  {
    int bushIndex = bushIndices[j];
    if(bushIndex < 0)
    {
      customizer->hasVirtualBushJoints = false;
    }
    else
    {
      JointValSet & jointValSet = customizer->jointValSets[j];
      jointValSet.valuePtr = bodyInterface->getJointValuePtr(body, bushIndex);
      jointValSet.velocityPtr = bodyInterface->getJointVelocityPtr(body, bushIndex);
      jointValSet.torqueForcePtr = bodyInterface->getJointForcePtr(body, bushIndex);
    }
  }
}

static BodyCustomizerHandle create(BodyHandle bodyHandle, const char * modelName)
{
  PandaCustomizer * customizer = 0;

  int modelType = 0;
  string name(modelName);

  if(name == "Panda")
  {
    modelType = Panda;
  }

  if(modelType)
  {

    customizer = new PandaCustomizer;

    customizer->modelType = modelType;
    customizer->bodyHandle = bodyHandle;
    customizer->hasVirtualBushJoints = false;

    switch(modelType)
    {

      case Panda:
        customizer->springT = 3.0e4;
        customizer->dampingT = 2.0e2;
        customizer->springR = 124;
        customizer->dampingR = 2.5;

        getVirtualbushJoints(customizer, bodyHandle);

        break;

      default:
        break;
    }
  }

  return static_cast<BodyCustomizerHandle>(customizer);
}

static void destroy(BodyCustomizerHandle customizerHandle)
{
  PandaCustomizer * customizer = static_cast<PandaCustomizer *>(customizerHandle);
  if(customizer)
  {
    delete customizer;
  }
}

static void setVirtualJointForces(BodyCustomizerHandle customizerHandle)
{
  PandaCustomizer * customizer = static_cast<PandaCustomizer *>(customizerHandle);

  if(customizer->hasVirtualBushJoints)
  {

    for(int j = 0; j < 3; ++j)
    {
      JointValSet & trans = customizer->jointValSets[j];
      *(trans.torqueForcePtr) = -customizer->springT * (*trans.valuePtr) - customizer->dampingT * (*trans.velocityPtr);
    }

    for(int j = 3; j < 6; ++j)
    {
      JointValSet & rot = customizer->jointValSets[j];
      *(rot.torqueForcePtr) = -customizer->springR * (*rot.valuePtr) - customizer->dampingR * (*rot.velocityPtr);
    }
  }
}

extern "C" DLL_EXPORT NS_HRPMODEL::BodyCustomizerInterface * getHrpBodyCustomizerInterface(
    NS_HRPMODEL::BodyInterface * bodyInterface_)
{
  bodyInterface = bodyInterface_;

  bodyCustomizerInterface.version = NS_HRPMODEL::BODY_CUSTOMIZER_INTERFACE_VERSION;
  bodyCustomizerInterface.getTargetModelNames = getTargetModelNames;
  bodyCustomizerInterface.create = create;
  bodyCustomizerInterface.destroy = destroy;
  bodyCustomizerInterface.setVirtualJointForces = setVirtualJointForces;

  return &bodyCustomizerInterface;
}
