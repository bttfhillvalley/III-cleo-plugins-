#define _USE_MATH_DEFINES
#include <string>
#include <map>
#include "plugin.h"
#include "irrKlang.h"
#include "III.CLEO.h"
#pragma comment(lib, "III.CLEO.lib")
#include "CCamera.h"
#include "CTimer.h"
#include "CClumpModelInfo.h"
#include "CMenuManager.h"
#include "CMessages.h"
#include "CWorld.h"
#include "extensions\ScriptCommands.h"

#define GRAVITY (0.008f)
#define Clamp(v, low, high) ((v) < (low) ? (low) : (v) > (high) ? (high) : (v))
#define SQR(x) ((x) * (x))

using namespace irrklang;
using namespace plugin;
using namespace std;

tScriptVar* Params;

int visibility;
boolean paused = false;

ISoundEngine* m_soundEngine;
bool loadedSound = false;
char volume = 0;

struct GameSound {
	ISound* sound;
	CVehicle* vehicle;
	CVector offset{ 0.0,0.0,0.0 };
	bool spatial;
};

struct CarAttachments {
	CVehicle* vehicle;
	CVehicle* attached;
	CVector offset{ 0.0, 0.0, 0.0 };
	// TODO Rotation
};

map<string, GameSound> soundMap;

inline float DotProduct(const CVector& v1, const CVector& v2)
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

struct tFlyingHandlingData {
	int m_dwVehicleId;
	float fThrust;
	float fThrustFallOff;
	float fYaw;
	float fYawStab;
	float fSideSlip;
	float fRoll;
	float fRollStab;
	float fPitch;
	float fPitchStab;
	float fFormLift;
	float fAttackLift;
	float fMoveRes;
	CVector vecTurnRes;
	CVector vecSpeedRes;
};

tFlyingHandlingData *flyingHandling;

RpMaterial* __cdecl SetAmbientCB(RpMaterial* material, void* data)
{
	RwSurfaceProperties* properties = (RwSurfaceProperties*)RpMaterialGetSurfaceProperties(material);
	if (data == (void*)(0))
		properties->ambient = 0.5f;
	else
		properties->ambient = 5.0f;
	return material;
}

RpMaterial* SetRedCB(RpMaterial* material, void* data)
{
	unsigned int value = reinterpret_cast<unsigned int>(data);
	RwRGBA* col = (RwRGBA*)RpMaterialGetColor(material);	// get rid of const
	RwSurfaceProperties* properties = (RwSurfaceProperties*)RpMaterialGetSurfaceProperties(material);
	col->red = value;
	return material;
}

RpMaterial* SetGreenCB(RpMaterial* material, void* data)
{
	unsigned int value = reinterpret_cast<unsigned int>(data);
	RwRGBA* col = (RwRGBA*)RpMaterialGetColor(material);	// get rid of const
	RwSurfaceProperties* properties = (RwSurfaceProperties*)RpMaterialGetSurfaceProperties(material);
	col->green = value;
	return material;
}

RpMaterial* SetBlueCB(RpMaterial* material, void* data)
{
	unsigned int value = reinterpret_cast<unsigned int>(data);
	RwRGBA* col = (RwRGBA*)RpMaterialGetColor(material);	// get rid of const
	RwSurfaceProperties* properties = (RwSurfaceProperties*)RpMaterialGetSurfaceProperties(material);
	col->blue = value;
	return material;
}

RpMaterial* SetCompAlphaCB(RpMaterial* material, void* data)
{
	unsigned int alpha = (unsigned int)(uintptr_t)data;
	RwRGBA* col = (RwRGBA*)RpMaterialGetColor(material);	// get rid of const
	col->alpha = alpha;
	return material;
}

RwObject* __cdecl SetAtomicVisibilityCB(RwObject* rwObject, void* data) {
	if (data == (void*)(0))
		rwObject->flags = 0;
	else
		rwObject->flags = 4;
	return rwObject;
}

RwObject* __cdecl GetAtomicVisibilityCB(RwObject* rwObject, void* data) {
	visibility = (int)rwObject->flags;
	return rwObject;
}

RwObject* __cdecl GetAtomicObjectCB(RwObject* object, void* data)
{
	*(RpAtomic**)data = (RpAtomic*)object;
	return object;
}

void ApplyTurnForce(CVehicle* vehicle, const CVector &j, const CVector &p) { vehicle->ApplyTurnForce(j.x, j.y, j.z, p.x, p.y, p.z); }
void ApplyMoveForce(CVehicle* vehicle, const CVector &j) { vehicle->ApplyMoveForce(j.x, j.y, j.z); }
float radians(float degrees) { return (float)(degrees * M_PI / 180.0); }
float degrees(float radians) { return (float)(radians * 180.0 / M_PI); }

void HoverControl(CVehicle* vehicle)
{
	float fThrust = 0.0f;
	float fPitch = 0.0f;
	float fRoll = 0.0f;
	float fYaw = 0.0f;
	//tFlyingHandlingData* flyingHandling;
	float rm = pow(flyingHandling->fMoveRes, CTimer::ms_fTimeStep);
	/*if (vehicle->m_nState != 0 && vehicle->m_nState != 10) {
		rm *= 0.97f;
	}*/
	vehicle->m_vecMoveSpeed *= rm;
	float fUpSpeed = DotProduct(vehicle->m_vecMoveSpeed, vehicle->m_matrix.at);
	if (vehicle->m_nState == 0 || vehicle->m_nState == 10) {
		fThrust = (CPad::GetPad(0)->GetAccelerate() - CPad::GetPad(0)->GetBrake()) / 255.0f;
		
		fRoll = -CPad::GetPad(0)->GetSteeringLeftRight() / 128.0f;
		fPitch = CPad::GetPad(0)->GetSteeringUpDown() / 128.0f;
		if (abs(CPad::GetPad(0)->GetCarGunUpDown()) > 1.0f && abs(CPad::GetPad(0)->LookAroundUpDown()))
			fThrust = CPad::GetPad(0)->GetCarGunUpDown() / 128.0f;
		fYaw = CPad::GetPad(0)->GetCarGunLeftRight() / 128.0f;

		/*if (CPad::GetPad(0)->GetCarGunFired()) {
			fYaw = 0.0f;
			fPitch = Clamp(0.5f * DotProduct(vehicle->m_vecMoveSpeed, vehicle->m_matrix.up), -0.1f, 0.1f);
			fRoll = Clamp(0.5f * DotProduct(vehicle->m_vecMoveSpeed, vehicle->m_matrix.right), -0.1f, 0.1f);
		}*/
	}
	/*else {
		fThrust = -0.1f;
		fYaw = 0.0f;
		fPitch = Clamp(0.5f * DotProduct(vehicle->m_vecMoveSpeed, vehicle->m_matrix.up), -0.1f, 0.1f);
		fRoll = Clamp(0.5f * DotProduct(vehicle->m_vecMoveSpeed, vehicle->m_matrix.right), -0.1f, 0.1f);
	}*/
	if (fThrust < 0.0f)
		fThrust *= 2.0f;
	fThrust = flyingHandling->fThrust * fThrust + 0.95f;
	fThrust -= flyingHandling->fThrustFallOff * fUpSpeed;
	if (vehicle->GetPosition().z > 1000.0f)
		fThrust *= 10.0f / (vehicle->GetPosition().z - 70.0f);
	ApplyMoveForce(vehicle, GRAVITY * vehicle->m_matrix.at * fThrust * vehicle->m_fMass * CTimer::ms_fTimeStep);

	if (vehicle->m_matrix.at.z > 0.0f) {
		float upRight = Clamp(vehicle->m_matrix.right.z, -flyingHandling->fFormLift, flyingHandling->fFormLift);
		float upImpulseRight = -upRight * flyingHandling->fAttackLift * vehicle->m_fTurnMass * CTimer::ms_fTimeStep;
		ApplyTurnForce(vehicle, upImpulseRight * vehicle->m_matrix.at, vehicle->m_matrix.right);

		float upFwd = Clamp(vehicle->m_matrix.up.z, -flyingHandling->fFormLift, flyingHandling->fFormLift);
		float upImpulseFwd = -upFwd * flyingHandling->fAttackLift * vehicle->m_fTurnMass * CTimer::ms_fTimeStep;
		ApplyTurnForce(vehicle, upImpulseFwd * vehicle->m_matrix.at, vehicle->m_matrix.up);
	}
	else {
		float upRight = vehicle->m_matrix.right.z < 0.0f ? -flyingHandling->fFormLift : flyingHandling->fFormLift;
		float upImpulseRight = -upRight * flyingHandling->fAttackLift * vehicle->m_fTurnMass * CTimer::ms_fTimeStep;
		ApplyTurnForce(vehicle, upImpulseRight * vehicle->m_matrix.at, vehicle->m_matrix.right);

		float upFwd = vehicle->m_matrix.up.z < 0.0f ? -flyingHandling->fFormLift : flyingHandling->fFormLift;
		float upImpulseFwd = -upFwd * flyingHandling->fAttackLift * vehicle->m_fTurnMass * CTimer::ms_fTimeStep;
		ApplyTurnForce(vehicle, upImpulseFwd * vehicle->m_matrix.at, vehicle->m_matrix.up);
	}

	ApplyTurnForce(vehicle, fPitch * vehicle->m_matrix.at * flyingHandling->fPitch * vehicle->m_fTurnMass * CTimer::ms_fTimeStep, vehicle->m_matrix.up);
	ApplyTurnForce(vehicle, fRoll * vehicle->m_matrix.at * flyingHandling->fRoll * vehicle->m_fTurnMass * CTimer::ms_fTimeStep, vehicle->m_matrix.right);

	float fSideSpeed = -DotProduct(vehicle->m_vecMoveSpeed, vehicle->m_matrix.right);
	float fSideSlipAccel = flyingHandling->fSideSlip * fSideSpeed * abs(fSideSpeed);
	ApplyMoveForce(vehicle, vehicle->m_fMass * vehicle->m_matrix.right * fSideSlipAccel * CTimer::ms_fTimeStep);
	float fYawAccel = flyingHandling->fYawStab * fSideSpeed * abs(fSideSpeed) + flyingHandling->fYaw * fYaw;
	ApplyTurnForce(vehicle, fYawAccel * vehicle->m_matrix.right * vehicle->m_fTurnMass * CTimer::ms_fTimeStep, -vehicle->m_matrix.up);
	ApplyTurnForce(vehicle, fYaw * vehicle->m_matrix.up * flyingHandling->fYaw * vehicle->m_fTurnMass * CTimer::ms_fTimeStep, vehicle->m_matrix.right);

	float rX = pow(flyingHandling->vecTurnRes.x, CTimer::ms_fTimeStep);
	float rY = pow(flyingHandling->vecTurnRes.y, CTimer::ms_fTimeStep);
	float rZ = pow(flyingHandling->vecTurnRes.z, CTimer::ms_fTimeStep);
	CVector vecTurnSpeed = Multiply3x3(vehicle->m_vecTurnSpeed, vehicle->m_matrix);
	float fResistanceMultiplier = powf(1.0f / (flyingHandling->vecSpeedRes.z * SQR(vecTurnSpeed.z) + 1.0f) * rZ, CTimer::ms_fTimeStep);
	float fResistance = vecTurnSpeed.z * fResistanceMultiplier - vecTurnSpeed.z;
	vecTurnSpeed.x *= rX;
	vecTurnSpeed.y *= rY;
	vecTurnSpeed.z *= fResistanceMultiplier;
	vehicle->m_vecTurnSpeed = Multiply3x3(vehicle->m_matrix, vecTurnSpeed);
	ApplyTurnForce(vehicle, -vehicle->m_matrix.right * fResistance * vehicle->m_fTurnMass, vehicle->m_matrix.up + Multiply3x3(vehicle->m_matrix, vehicle->m_vecCentreOfMass));
}

eOpcodeResult __stdcall setHover(CScript* script)
{
	script->Collect(2);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	if (vehicle) {
		HoverControl(vehicle);
	}
	return OR_CONTINUE;
}

void setVisibility(CEntity* model, char* component, int visibility) {
	RwFrame* frame = CClumpModelInfo::GetFrameFromName(model->m_pRwClump, component);
	if (frame) {
		RwFrameForAllObjects(frame, SetAtomicVisibilityCB, (void*)visibility);
	}
}

void getVisibility(CEntity* model, char* component) {
	visibility = 0;
	RwFrame* frame = CClumpModelInfo::GetFrameFromName(model->m_pRwClump, component);
	if (frame) {
		RwFrameForAllObjects(frame, GetAtomicVisibilityCB, NULL);
	}
}

void moveComponent(CEntity* model, char* component, float x, float y, float z) {
	RwFrame* frame = CClumpModelInfo::GetFrameFromName(model->m_pRwClump, component);
	if (frame) {
		CMatrix cmmatrix(&frame->modelling, false);
		cmmatrix.SetTranslateOnly(x, y, z);
		cmmatrix.UpdateRW();
	}
}

void rotateComponent(CEntity* model, char* component, float rx, float ry, float rz) {
	RwFrame* frame = CClumpModelInfo::GetFrameFromName(model->m_pRwClump, component);
	if (frame) {
		CMatrix cmatrix(&frame->modelling, false);
		CVector cpos(cmatrix.pos);
		cmatrix.SetRotate(radians(rx), radians(ry), radians(rz));
		cmatrix.pos = cpos;
		cmatrix.UpdateRW();
	}
}

void setColor(CVehicle* vehicle, char* component, int red, int green, int blue) {
	RwFrame* frame = CClumpModelInfo::GetFrameFromName(vehicle->m_pRwClump, component);
	if (frame) {
		RpAtomic* atomic;
		RpGeometry* geometry;
		RwFrameForAllObjects(frame, GetAtomicObjectCB, &atomic);
		geometry = atomic->geometry;
		RpGeometryForAllMaterials(geometry, SetRedCB, (void*)red);
		RpGeometryForAllMaterials(geometry, SetGreenCB, (void*)green);
		RpGeometryForAllMaterials(geometry, SetBlueCB, (void*)blue);
	}
}

void setAlpha(CVehicle* vehicle, char* component, int alpha) {
	RwFrame* frame = CClumpModelInfo::GetFrameFromName(vehicle->m_pRwClump, component);
	if (frame) {
		RpAtomic* atomic;
		RpGeometry* geometry;
		RwFrameForAllObjects(frame, GetAtomicObjectCB, &atomic);
		geometry = atomic->geometry;
		geometry->flags |= rpGEOMETRYMODULATEMATERIALCOLOR;
		RpGeometryForAllMaterials(geometry, SetCompAlphaCB, (void*)alpha);
		RwFrameForAllObjects(frame, SetAtomicVisibilityCB, (void*)alpha);
	}
}

RwUInt8 getAlpha(CVehicle* vehicle, char* component) {
	RwFrame* frame = CClumpModelInfo::GetFrameFromName(vehicle->m_pRwClump, component);
	RwUInt8 alpha = 0;
	if (frame) {
		RpAtomic* atomic;
		RpGeometry* geometry;
		RwFrameForAllObjects(frame, GetAtomicObjectCB, &atomic);
		geometry = atomic->geometry;
		alpha = atomic->geometry->matList.materials[0]->color.alpha;
	}
	return alpha;
}

void setGlow(CVehicle* vehicle, char* component, int glow) {
	RwFrame* frame = CClumpModelInfo::GetFrameFromName(vehicle->m_pRwClump, component);
	if (frame) {
		RpAtomic* atomic;
		RpGeometry* geometry;
		RwFrameForAllObjects(frame, GetAtomicObjectCB, &atomic);
		geometry = atomic->geometry;
		RpGeometryForAllMaterials(geometry, SetAmbientCB, (void*)glow);
	}
}

// Opcodes
eOpcodeResult __stdcall setCarComponentVisibility(CScript* script)
{
	script->Collect(3);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	setVisibility(vehicle, Params[1].cVar, Params[2].nVar);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall getCarComponentVisibility(CScript* script)
{
	script->Collect(2);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	getVisibility(vehicle, Params[1].cVar);
	Params[0].nVar = visibility;
	script->Store(1);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall setCarComponentGlow(CScript* script)
{
	script->Collect(3);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	setGlow(vehicle, Params[1].cVar, Params[2].nVar);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall setCarComponentGlowIndex(CScript* script)
{
	script->Collect(4);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	char component[256];
	sprintf(component, "%s%d", Params[1].cVar, Params[3].nVar);
	setGlow(vehicle, component, Params[2].nVar);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall setCarComponentIndexVisibility(CScript* script)
{
	script->Collect(4);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	char component[256];
	sprintf(component, "%s%d", Params[1].cVar, Params[3].nVar);
	setVisibility(vehicle, component, Params[2].nVar);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall setCarComponentColor(CScript* script)
{
	script->Collect(5);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	setColor(vehicle, Params[1].cVar, Params[2].nVar, Params[3].nVar, Params[4].nVar);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall setCarComponentIndexColor(CScript* script)
{
	script->Collect(6);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	char component[256];
	sprintf(component, "%s%d", Params[1].cVar, Params[5].nVar);
	setColor(vehicle, component, Params[2].nVar, Params[3].nVar, Params[4].nVar);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall setCarComponentAlpha(CScript* script)
{
	script->Collect(3);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	setAlpha(vehicle, Params[1].cVar, Params[2].nVar);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall setCarComponentIndexAlpha(CScript* script)
{
	script->Collect(4);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	char component[256];
	sprintf(component, "%s%d", Params[1].cVar, Params[3].nVar);
	setAlpha(vehicle, component, Params[2].nVar);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall getCarComponentAlpha(CScript* script)
{
	script->Collect(2);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	Params[0].nVar = getAlpha(vehicle, Params[1].cVar);
	script->Store(1);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall getCarComponentIndexAlpha(CScript* script)
{
	script->Collect(3);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	char component[256];
	sprintf(component, "%s%d", Params[1].cVar, Params[3].nVar);
	Params[0].nVar = getAlpha(vehicle, component);
	script->Store(1);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall moveCarComponent(CScript* script)
{
	script->Collect(5);
	CVehicle* vehicle = CPools::GetVehicle(Params[4].nVar);
	moveComponent(vehicle, Params[0].cVar, Params[1].fVar, Params[2].fVar, Params[3].fVar);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall moveCarComponentIndex(CScript* script)
{
	script->Collect(6);
	CVehicle* vehicle = CPools::GetVehicle(Params[4].nVar);
	char component[256];
	sprintf(component, "%s%d", Params[0].cVar, Params[5].nVar);
	moveComponent(vehicle, component, Params[1].fVar, Params[2].fVar, Params[3].fVar);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall getCarComponentPosition(CScript* script)
{
	script->Collect(2);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	RwFrame* frame = CClumpModelInfo::GetFrameFromName(vehicle->m_pRwClump, Params[1].cVar);
	if (frame) {
		CMatrix cmatrix(&frame->modelling, false);
		Params[0].fVar = cmatrix.pos.x;
		Params[1].fVar = cmatrix.pos.y;
		Params[2].fVar = cmatrix.pos.z;
	}
	script->Store(3);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall getCarComponentOffset(CScript* script)
{
	script->Collect(5);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	RwFrame* frame = CClumpModelInfo::GetFrameFromName(vehicle->m_pRwClump, Params[1].cVar);
	if (frame) {
		CMatrix cmatrix(&frame->modelling, false);
		CVector offset = CVector(Params[2].fVar, Params[3].fVar, Params[4].fVar);
		CVector coords = Multiply3x3(vehicle->m_matrix, Multiply3x3(cmatrix, offset) + cmatrix.pos) + vehicle->GetPosition();
		Params[0].fVar = coords.x;
		Params[1].fVar = coords.y;
		Params[2].fVar = coords.z;
	}
	script->Store(3);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall rotateCarComponent(CScript* script)
{
	script->Collect(5);
	CVehicle* vehicle = CPools::GetVehicle(Params[4].nVar);
	rotateComponent(vehicle, Params[0].cVar, Params[1].fVar, Params[2].fVar, Params[3].fVar);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall rotateCarComponentIndex(CScript* script)
{
	script->Collect(5);
	CVehicle* vehicle = CPools::GetVehicle(Params[4].nVar);
	char component[256];
	sprintf(component, "%s%d", Params[0].cVar, Params[5].nVar);
	rotateComponent(vehicle, Params[0].cVar, Params[1].fVar, Params[2].fVar, Params[3].fVar);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall getCarComponentRotation(CScript* script)
{
	script->Collect(2);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	RwFrame* frame = CClumpModelInfo::GetFrameFromName(vehicle->m_pRwClump, Params[1].cVar);
	if (frame) {
		CMatrix cmatrix(&frame->modelling, false);
		float sy = sqrtf(cmatrix.right.x * cmatrix.right.x + cmatrix.up.x * cmatrix.up.x);
		bool singular = sy < 1e-6;

		if (!singular) {
			Params[0].fVar = degrees(atan2f(cmatrix.at.y, cmatrix.at.z));
			Params[1].fVar = degrees(atan2f(-cmatrix.at.x, sy));
			Params[2].fVar = degrees(atan2f(cmatrix.up.x, cmatrix.right.x));
		}
		else {
			Params[0].fVar = degrees(atan2f(-cmatrix.up.z, cmatrix.up.y));
			Params[1].fVar = degrees(atan2f(-cmatrix.at.x, sy));
			Params[2].fVar = 0.0f;
		}
	}
	script->Store(3);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall isDoorOpen(CScript* script) {
	script->Collect(2);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	if (vehicle) {

		Params[0].nVar = (int)vehicle->IsDoorClosed(static_cast<eDoors>(Params[1].nVar));
		script->Store(1);
	}
	return OR_CONTINUE;
}

eOpcodeResult __stdcall getWheelStatus(CScript* script)
{
	script->Collect(1);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	CAutomobile* automobile;
	int status = -1;
	if (vehicle) {
		automobile = reinterpret_cast<CAutomobile*>(vehicle);
		status = automobile->m_carDamage.GetWheelStatus(0);
	}
	Params[0].nVar = status;
	script->Store(1);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall setWheelStatus(CScript* script)
{
	script->Collect(2);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	CAutomobile* automobile;
	if (vehicle) {
		automobile = reinterpret_cast<CAutomobile*>(vehicle);
		int status = Params[1].nVar;
		automobile->m_carDamage.SetWheelStatus(0, status);
		automobile->m_carDamage.SetWheelStatus(1, status);
		automobile->m_carDamage.SetWheelStatus(2, status);
		automobile->m_carDamage.SetWheelStatus(3, status);
		if (status == 0) {
			automobile->m_carDamage.m_fWheelDamageEffect = 0.0f;
		}
	}
	return OR_CONTINUE;
}

eOpcodeResult __stdcall getEngineStatus(CScript* script)
{
	script->Collect(1);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	int status = 0;
	if (vehicle) {
		status = vehicle->m_nState == 0 || vehicle->m_nState == 10;
	}
	Params[0].nVar = status;
	script->Store(1);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall applyForwardForce(CScript* script)
{
	script->Collect(2);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);

	if (vehicle) {
		CVector force = vehicle->m_matrix.up * Params[1].fVar;
		vehicle->ApplyMoveForce(force.x, force.y, force.z);
	}
	return OR_CONTINUE;
}

eOpcodeResult __stdcall applyUpwardForce(CScript* script)
{
	script->Collect(2);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);

	if (vehicle) {
		CVector force = vehicle->m_matrix.at * Params[1].fVar;
		vehicle->ApplyMoveForce(force.x, force.y, force.z);
	}
	return OR_CONTINUE;
}

eOpcodeResult __stdcall inRemote(CScript* script)
{
	script->Collect(1);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	CPlayerInfo player = CWorld::Players[CWorld::PlayerInFocus];
	static char  message[128];


	if (vehicle) {
		CAutomobile* automobile = reinterpret_cast<CAutomobile*>(vehicle);
		snprintf(message, 128, "Remote: %d, State: %d, this is a long string",
			player.IsPlayerInRemoteMode(),
			automobile->m_nState
		);
		//CMessages::AddMessageJumpQ(message, 150, 0);
		if (player.IsPlayerInRemoteMode() && automobile->m_nState == 10) {

			script->UpdateCompareFlag(1);
			return OR_CONTINUE;
		}
	}
	script->UpdateCompareFlag(0);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall setCarStatus(CScript* script)
{
	script->Collect(2);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	vehicle->m_nState = Params[1].nVar & 0xFF;
	return OR_CONTINUE;
}

eOpcodeResult __stdcall getCarStatus(CScript* script)
{
	script->Collect(1);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	Params[0].nVar = vehicle->m_nState;
	script->Store(1);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall setCarLights(CScript* script)
{
	script->Collect(2);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	vehicle->m_nVehicleFlags.bLightsOn = Params[1].nVar != 0;
	return OR_CONTINUE;
}

eOpcodeResult __stdcall getCarLights(CScript* script)
{
	script->Collect(1);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	Params[0].nVar = vehicle->m_nVehicleFlags.bLightsOn;
	script->Store(1);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall getWheelAngle(CScript* script) {
	script->Collect(1);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	if (vehicle) {
		CAutomobile* automobile = reinterpret_cast<CAutomobile*>(vehicle);
		for (int n = 0; n < 4; n++) {
			Params[n].fVar = degrees(automobile->m_afWheelRotation[n]);
		}
		script->Store(4);
	}
	return OR_CONTINUE;
}

eOpcodeResult __stdcall getSteeringAngle(CScript* script)
{
	script->Collect(1);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	CAutomobile* automobile;
	float angle = 0.0;
	if (vehicle) {
		automobile = reinterpret_cast<CAutomobile*>(vehicle);
		angle = degrees(automobile->m_fSteerAngle);
	}
	Params[0].fVar = angle;
	script->Store(1);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall getDriveWheelsOnGround(CScript* script) {
	script->Collect(1);
	CVehicle* vehicle = CPools::GetVehicle(Params[0].nVar);
	if (vehicle) {
		CAutomobile* automobile = reinterpret_cast<CAutomobile*>(vehicle);
		Params[0].nVar = (int)automobile->m_nDriveWheelsOnGround;
		script->Store(1);
	}
	return OR_CONTINUE;
}

bool wheelsOnGround(int vehiclePointer) {
	CVehicle* vehicle = CPools::GetVehicle(vehiclePointer);
	if (vehicle) {
		CAutomobile* automobile = reinterpret_cast<CAutomobile*>(vehicle);
		return automobile->m_nWheelsOnGround > 0;
	}
	return false;
}

eOpcodeResult __stdcall isWheelsOnGround(CScript* script) {
	script->Collect(1);
	script->UpdateCompareFlag(wheelsOnGround(Params[0].nVar));
	return OR_CONTINUE;
}

eOpcodeResult __stdcall isWheelsNotOnGround(CScript* script) {
	script->Collect(1);
	script->UpdateCompareFlag(!wheelsOnGround(Params[0].nVar));
	return OR_CONTINUE;
}

// Sound

// Sound helper methods
void cleanupSound(string key) {
	if (soundMap.contains(key)) {
		soundMap[key].sound->stop();
		soundMap[key].sound->drop();
		soundMap.erase(key);
	}
}

string getKeyIndex(char* name, int index) {
	string key(name);
	return key + "_" + to_string(index);
}

int findEmptyIndex(char* name) {
	string key;
	int index = 0;
	do {
		key = getKeyIndex(name, ++index);
	} while (soundMap.contains(key));
	return index;
}

// Sound opcodes
eOpcodeResult __stdcall stopAllSounds(CScript* script)
{
	script->Collect(0);
	m_soundEngine->stopAllSounds();
	return OR_CONTINUE;
}

eOpcodeResult __stdcall stopSound(CScript* script)
{
	script->Collect(1);
	string key(Params[0].cVar);
	cleanupSound(key);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall stopSoundIndex(CScript* script)
{
	script->Collect(2);
	string key = getKeyIndex(Params[0].cVar, Params[1].nVar);
	cleanupSound(key);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall isSoundPlaying(CScript* script)
{
	script->Collect(1);
	string key(Params[0].cVar);
	script->UpdateCompareFlag(soundMap.contains(key) && !soundMap[key].sound->isFinished());
	return OR_CONTINUE;
}

eOpcodeResult __stdcall isSoundStopped(CScript* script)
{
	script->Collect(1);
	string key(Params[0].cVar);
	script->UpdateCompareFlag(!(soundMap.contains(key) && !soundMap[key].sound->isFinished()));
	return OR_CONTINUE;
}

eOpcodeResult __stdcall isSoundPlayingIndex(CScript* script)
{
	script->Collect(2);
	string key = getKeyIndex(Params[0].cVar, Params[1].nVar);
	script->UpdateCompareFlag(soundMap.contains(key) && !soundMap[key].sound->isFinished());
	return OR_CONTINUE;
}

eOpcodeResult __stdcall isSoundStoppedIndex(CScript* script)
{
	script->Collect(2);
	string key = getKeyIndex(Params[0].cVar, Params[1].nVar);
	script->UpdateCompareFlag(!(soundMap.contains(key) && !soundMap[key].sound->isFinished()));
	return OR_CONTINUE;
}

void __playSound(string key) {
	char fullpath[128];
	snprintf(fullpath, 128, ".\\sound\\%s", Params[0].cVar);
	cleanupSound(key);
	soundMap[key].sound = m_soundEngine->play2D(fullpath, Params[1].nVar, false, true);
	soundMap[key].spatial = false;
}

eOpcodeResult __stdcall playSound(CScript* script)
{
	script->Collect(2);
	string key(Params[0].cVar);
	__playSound(key);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall playSoundIndex(CScript* script)
{
	script->Collect(2);
	int index = findEmptyIndex(Params[0].cVar);
	string key = getKeyIndex(Params[0].cVar, index);
	__playSound(key);
	Params[0].nVar = index;
	script->Store(1);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall playKeypad(CScript* script)
{
	script->Collect(1);
	char fullpath[128];
	snprintf(fullpath, 128, ".\\sound\\%d.wav", Params[0].nVar);
	m_soundEngine->play2D(fullpath);
	return OR_CONTINUE;
}

void __playSoundLocation(string key) {
	char fullpath[128];
	snprintf(fullpath, 128, ".\\sound\\%s", Params[0].cVar);
	cleanupSound(key);
	vec3df pos;
	pos.X = Params[1].fVar;
	pos.Y = -1.0f * Params[2].fVar;
	pos.Z = Params[3].fVar;
	soundMap[key].offset = CVector(pos.X, pos.Y, pos.Z);
	soundMap[key].sound = m_soundEngine->play3D(fullpath, pos, Params[4].nVar, false, true);
	soundMap[key].sound->setMinDistance(Params[5].fVar);
	soundMap[key].spatial = true;
}

eOpcodeResult __stdcall playSoundAtLocation(CScript* script)
{
	script->Collect(6);
	string key(Params[0].cVar);
	__playSoundLocation(key);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall playSoundAtLocationIndex(CScript* script)
{
	script->Collect(6);
	int index = findEmptyIndex(Params[0].cVar);
	string key = getKeyIndex(Params[0].cVar, index);
	__playSoundLocation(key);
	Params[0].nVar = index;
	script->Store(1);
	return OR_CONTINUE;
}

void __attachSoundToVehicle(string key, CVehicle* vehicle) {
	char fullpath[128];
	snprintf(fullpath, 128, ".\\sound\\%s", Params[0].cVar);
	cleanupSound(key);
	vec3df pos;
	soundMap[key].vehicle = vehicle;
	soundMap[key].offset = CVector(Params[1].fVar, Params[2].fVar, Params[3].fVar);
	Command<Commands::GET_OFFSET_FROM_CAR_IN_WORLD_COORDS>(vehicle, soundMap[key].offset.x, soundMap[key].offset.y, soundMap[key].offset.z, &pos.X, &pos.Y, &pos.Z);
	pos.Y *= -1.0;
	soundMap[key].sound = m_soundEngine->play3D(fullpath, pos, Params[4].nVar, false, true);
	soundMap[key].sound->setMinDistance(Params[5].fVar);
	soundMap[key].spatial = true;
}

eOpcodeResult __stdcall attachSoundToVehicle(CScript* script)
{
	script->Collect(7);
	int index = 0;
	CVehicle* vehicle = CPools::GetVehicle(Params[6].nVar);

	if (vehicle) {
		string key = getKeyIndex(Params[0].cVar, Params[6].nVar);
		__attachSoundToVehicle(key, vehicle);
	}
	return OR_CONTINUE;
}

eOpcodeResult __stdcall setFrequency(CScript* script)
{
	script->Collect(3);
	string key = getKeyIndex(Params[0].cVar, Params[1].nVar);
	if (soundMap.contains(key)) {
		soundMap[key].sound->setPlaybackSpeed(Params[2].fVar);
	}
	return OR_CONTINUE;
}

eOpcodeResult __stdcall setVolume(CScript* script)
{
	script->Collect(3);
	string key = getKeyIndex(Params[0].cVar, Params[1].nVar);
	if (soundMap.contains(key)) {
		soundMap[key].sound->setVolume(Params[2].fVar);
	}
	return OR_CONTINUE;
}

eOpcodeResult __stdcall attachSoundToVehicleIndex(CScript* script)
{
	script->Collect(7);
	int index = 0;
	CVehicle* vehicle = CPools::GetVehicle(Params[6].nVar);
	if (vehicle) {
		index = findEmptyIndex(Params[0].cVar);
		string key = getKeyIndex(Params[0].cVar, index);
		__attachSoundToVehicle(key, vehicle);
	}
	Params[0].nVar = index;
	script->Store(1);
	return OR_CONTINUE;
}

eOpcodeResult __stdcall setDoppler(CScript* script)
{
	script->Collect(2);
	m_soundEngine->setDopplerEffectParameters(Params[0].fVar, Params[1].fVar);
	return OR_CONTINUE;
}

BOOL APIENTRY DllMain(HMODULE hModule, DWORD reason, LPVOID lpReserved)
{
	static char message[256];
	CVector* pos;
	vec3df playerPos, soundPos, soundVel, dir;
	string key;
	float distance;
	if (reason == DLL_PROCESS_ATTACH)
	{
		DisableThreadLibraryCalls((HMODULE)hModule);
		Params = CLEO_GetParamsAddress();
		Opcodes::RegisterOpcode(0x3F02, getEngineStatus);
		Opcodes::RegisterOpcode(0x3F05, setHover);
		Opcodes::RegisterOpcode(0x3F06, isDoorOpen);
		Opcodes::RegisterOpcode(0x3F0E, getWheelAngle);
		Opcodes::RegisterOpcode(0x3F10, setCarComponentVisibility);
		Opcodes::RegisterOpcode(0x3F11, setCarComponentIndexVisibility);
		Opcodes::RegisterOpcode(0x3F12, setCarComponentAlpha);
		Opcodes::RegisterOpcode(0x3F13, setCarComponentIndexAlpha);
		Opcodes::RegisterOpcode(0x3F14, moveCarComponent);
		Opcodes::RegisterOpcode(0x3F15, moveCarComponentIndex);
		Opcodes::RegisterOpcode(0x3F16, rotateCarComponent);
		Opcodes::RegisterOpcode(0x3F17, rotateCarComponentIndex);
		Opcodes::RegisterOpcode(0x3F18, setCarComponentGlow);
		Opcodes::RegisterOpcode(0x3F19, setCarComponentGlowIndex);
		Opcodes::RegisterOpcode(0x3F1B, getCarComponentPosition);
		Opcodes::RegisterOpcode(0x3F1C, getCarComponentRotation);
		Opcodes::RegisterOpcode(0x3F1D, getCarComponentOffset);
		Opcodes::RegisterOpcode(0x3F25, applyForwardForce);
		Opcodes::RegisterOpcode(0x3F26, applyUpwardForce);
		Opcodes::RegisterOpcode(0x3F27, getWheelStatus);
		Opcodes::RegisterOpcode(0x3F28, setWheelStatus);
		Opcodes::RegisterOpcode(0x3F2A, inRemote);
		Opcodes::RegisterOpcode(0x3F2E, getDriveWheelsOnGround);
		Opcodes::RegisterOpcode(0x3F3E, isWheelsOnGround);
		Opcodes::RegisterOpcode(0x3F3F, isWheelsNotOnGround);
		Opcodes::RegisterOpcode(0x3F36, getSteeringAngle);
		Opcodes::RegisterOpcode(0x3F40, getCarComponentVisibility);
		Opcodes::RegisterOpcode(0x3F47, getCarComponentAlpha);
		Opcodes::RegisterOpcode(0x3F48, getCarComponentIndexAlpha);
		Opcodes::RegisterOpcode(0x3F49, setCarComponentColor);
		Opcodes::RegisterOpcode(0x3F4A, setCarComponentIndexColor);
		Opcodes::RegisterOpcode(0x3F80, stopAllSounds);
		Opcodes::RegisterOpcode(0x3F81, stopSound);
		Opcodes::RegisterOpcode(0x3F82, isSoundPlaying);
		Opcodes::RegisterOpcode(0x3F83, isSoundStopped);
		Opcodes::RegisterOpcode(0x3F84, playSound);
		Opcodes::RegisterOpcode(0x3F85, playSoundAtLocation);
		Opcodes::RegisterOpcode(0x3F86, attachSoundToVehicle);
		Opcodes::RegisterOpcode(0x3F90, playKeypad);
		Opcodes::RegisterOpcode(0x3F91, stopSoundIndex);
		Opcodes::RegisterOpcode(0x3F92, isSoundPlayingIndex);
		Opcodes::RegisterOpcode(0x3F93, isSoundStoppedIndex);
		Opcodes::RegisterOpcode(0x3F94, playSoundIndex);
		Opcodes::RegisterOpcode(0x3F95, playSoundAtLocationIndex);
		Opcodes::RegisterOpcode(0x3F96, attachSoundToVehicleIndex);
		Opcodes::RegisterOpcode(0x3F97, setFrequency);
		Opcodes::RegisterOpcode(0x3F98, setVolume);
		Opcodes::RegisterOpcode(0x3F99, setDoppler);
		Opcodes::RegisterOpcode(0x3F9A, setCarStatus);
		Opcodes::RegisterOpcode(0x3F9B, getCarStatus);
		Opcodes::RegisterOpcode(0x3F9C, setCarLights);
		Opcodes::RegisterOpcode(0x3F9D, getCarLights);
		Events::initGameEvent += [] {
			if (!loadedSound) {
				m_soundEngine = createIrrKlangDevice();
				m_soundEngine->setRolloffFactor(1.5f);
				m_soundEngine->setDopplerEffectParameters(2.0f, 10.0f);
				loadedSound = true;
			}
			else {
				m_soundEngine->removeAllSoundSources();
			}
			soundMap.clear();
			flyingHandling = new tFlyingHandlingData();
			flyingHandling->fThrust = 0.50f;
			flyingHandling->fThrustFallOff = 5.0f;
			flyingHandling->fYaw = -0.001f;
			flyingHandling->fYawStab = 0.0f;
			flyingHandling->fSideSlip = 0.1f;
			flyingHandling->fRoll = 0.0065f;
			flyingHandling->fRollStab = 0.0f;
			flyingHandling->fPitch = 0.0035f;
			flyingHandling->fPitchStab = 0.0f;
			flyingHandling->fFormLift = 0.0f;
			flyingHandling->fAttackLift = 0.0f;
			flyingHandling->fMoveRes = 0.999f;
			flyingHandling->vecTurnRes.x = 0.9f;
			flyingHandling->vecTurnRes.y = 0.9f;
			flyingHandling->vecTurnRes.z = 0.99f;
			flyingHandling->vecSpeedRes.x = 0.0f;
			flyingHandling->vecSpeedRes.y = 0.0f;
			flyingHandling->vecSpeedRes.z = 0.0f;
		};

		Events::gameProcessEvent += [&] {
			if (volume != FrontEndMenuManager.m_PrefsSfxVolume) {
				volume = FrontEndMenuManager.m_PrefsSfxVolume;				
				m_soundEngine->setSoundVolume(volume / 127.0f);
			}
			if (Command<Commands::IS_PLAYER_PLAYING>(0))
			{
				pos = TheCamera.GetGameCamPosition();
				playerPos.X = pos->x;
				playerPos.Y = -1.0f * pos->y;
				playerPos.Z = pos->z;
				dir.X = TheCamera.m_ViewMatrix.up.x;
				dir.Y = -1.0f * TheCamera.m_ViewMatrix.up.y;
				dir.Z = TheCamera.m_ViewMatrix.up.z;
				m_soundEngine->setListenerPosition(playerPos, dir, vec3df(0, 0, 0), vec3df(0, 0, 1));
			}
			if (!soundMap.empty()) {
				if (FrontEndMenuManager.m_bMenuActive && !paused) {

					for (auto const& [key, gamesound] : soundMap) {
						gamesound.sound->setIsPaused();
					}
					paused = true;
				}
				else if (!FrontEndMenuManager.m_bMenuActive) {
					auto itr = soundMap.begin();
					while (itr != soundMap.end()) {
						// Delete sound if its finished playing
						if (soundMap[itr->first].sound->isFinished()) {
							itr = soundMap.erase(itr);
							continue;
						}
						// Unpause sound if we're paused
						if (paused) {
							soundMap[itr->first].sound->setIsPaused(false);
						}
						if (soundMap[itr->first].vehicle) {
							// Stop sound if wrecked
							int index = CPools::GetVehicleRef(soundMap[itr->first].vehicle);
							if (index < 0 || soundMap[itr->first].vehicle->m_nState == 5) {
								soundMap[itr->first].sound->stop();
								soundMap[itr->first].sound->drop();
								itr = soundMap.erase(itr);
								continue;
							}
							// Attach sound to vehicle
							Command<0x04C3>(soundMap[itr->first].vehicle, soundMap[itr->first].offset.x, soundMap[itr->first].offset.y, soundMap[itr->first].offset.z, &soundPos.X, &soundPos.Y, &soundPos.Z);
							soundPos.Y *= -1.0;
							soundMap[itr->first].sound->setPosition(soundPos);
							// Set speed for doppler effect
							soundVel.X = soundMap[itr->first].vehicle->m_vecMoveSpeed.x;
							soundVel.Y = soundMap[itr->first].vehicle->m_vecMoveSpeed.y * -1.0f;
							soundVel.Z = soundMap[itr->first].vehicle->m_vecMoveSpeed.z;
							soundMap[itr->first].sound->setVelocity(soundVel);
						}
						else {
							// Set sound to specified location
							soundPos.X = soundMap[itr->first].offset.x;
							soundPos.Y = soundMap[itr->first].offset.y;
							soundPos.Z = soundMap[itr->first].offset.z;
						}

						// Mute sound if > 150 units away, otherwise play at full volume
						/*distance = (float)playerPos.getDistanceFrom(soundPos);
						if (distance < 150.0f || !soundMap[itr->first].spatial) {
							soundMap[itr->first].sound->setVolume(1.0f);
						}
						else {
							soundMap[itr->first].sound->setVolume(0.0f);
						}*/
						++itr;
					}
					paused = false;
				}
			}
		};
	}	
	return TRUE;
}
