#include <stdio.h>
#include <stdlib.h>
#include "physics.h"
#include "world.h"
#include "shaders.h"

typedef struct {
	float rhs[MAX_CONSTRAINTS*6];	
	int nRhs;
} LCP_t;

static void LCP_Solve(){
}

static void FixedConstraint_Evaluate(PhysicsConstraint_t *constraint);

void FixedConstraint_Create(PhysicsConstraint_t *constraint, Bone *body1, Bone *body2){

	constraint->body1 = body1;
	constraint->body2 = body2;

	if(body2){
		constraint->offset = Math_QuatRotate(Math_QuatInv(body2->absRot),
		Math_Vec3SubVec3(body1->absPos,body2->absPos));
		constraint->offsetRotation = Math_QuatMult(body1->absRot, Math_QuatInv(body2->absRot));
	} else {
		constraint->offset = body1->absPos;
		constraint->offsetRotation = body1->absRot;
	}
	constraint->Evaluate = FixedConstraint_Evaluate;
}


static void FixedConstraint_Evaluate(PhysicsConstraint_t *constraint){

	Vec3 offset, axis;
	Quat rotOffset;

	axis = Math_QuatRotate(constraint->body2->absRot,constraint->offset);
	offset = Math_Vec3AddVec3(axis, constraint->body2->absPos);
	rotOffset = Math_QuatMult(constraint->body2->absRot,constraint->offsetRotation);

	float identity[9] = {1,0,0,0,1,0,0,0,1};
	float identityminus[9] = {-1,0,0, 0,-1,0, 0,0,-1};
	float zeromatrix[9] = {0,0,0, 0,0,0, 0,0,0};
	//Math_Matrix6x6Set(constraint->J1, identity,zeromatrix,zeromatrix,identity);
	float skew[9];
	Math_SkewSymetricMat3(axis,skew);


	//Math_Matrix6x6Set(constraint->J2, identityminus, skew, zeromatrix, identityminus);

	constraint->c1[0] = Math_Vec3SubVec3(offset, constraint->body1->absPos);

	float angle;
	Vec3 angVel = Math_QuatToAxisAngle(Math_QuatMult(Math_QuatInv(constraint->body1->absRot),rotOffset), &angle);

	constraint->c1[1] = Math_Vec3MultFloat(angVel,angle);


	constraint->c2[0] = (Vec3){0,0,0};
	constraint->c2[1] = (Vec3){0,0,0};
}


void ConeConstraint_Create(PhysicsConstraint_t *constraint, Bone *body1,
								 Bone *body2, Vec3 axis, float coneAngle){
	constraint->cone.anchor = (Vec3){0,0,0};
	constraint->body1 = body1;
	constraint->body2 = body2;
	constraint->cone.angle = coneAngle;
	constraint->cone.axis =axis;
	constraint->cone.bodyAxis = (Vec3){0,1,0};
}

void ConeConstraint_Add(PhysicsConstraint_t *constraint){


	Vec3 body2pos = Math_Vec3AddVec3(constraint->body1->absPos, 
		Math_QuatRotate(constraint->body1->absRot, constraint->body2->absPos));
	Vec3 anchor = Math_Vec3AddVec3(constraint->cone.anchor,body2pos);
	Quat body2rot = Math_QuatMult(constraint->body1->absRot, constraint->body2->absRot);
	//Vec3 anchor = Math_Vec3AddVec3(constraint->body2->absPos, 
		//Math_QuatRotate(constraint->body2->absRot, constraint->cone.anchor);

	Vec3 body1Axis = Math_QuatRotate(constraint->body1->absRot, constraint->cone.bodyAxis); 
	Vec3 coneAxis = Math_QuatRotate(body2rot, constraint->cone.axis);	
	body1Axis = Math_Vec3Normalize(body1Axis);
	coneAxis = Math_Vec3Normalize(coneAxis);

	Vec3 normal = Math_Vec3Normalize(Math_Vec3Cross(body1Axis,coneAxis));
	//calculate the radius of a cone
	Quat q = Math_Quat(normal, constraint->cone.angle*0.5);
	Vec3 coneVector = Math_QuatRotate(q,coneAxis);
	
	normal = Math_Vec3Cross(Math_Vec3Cross(coneVector,body1Axis),coneVector);
	normal = Math_Vec3Normalize(normal);
	// Vec3 point= Math_Vec3MultVec3(constraint->body2->absPos, coneVector);

	Vec3 point = coneVector;

	if(Math_Vec3Dot(body1Axis,coneAxis) > cos(constraint->cone.angle*0.5)){
		Vec3 lines[] = {
		Math_Vec3AddVec3(constraint->body2->absPos,coneAxis),
		Math_Vec3AddVec3(constraint->body2->absPos,Math_Vec3MultFloat(coneAxis,10))};
		World_DrawLinesColor(lines, 2,(Vec4){0,0,1,1});
		lines[0] = Math_Vec3AddVec3(constraint->body2->absPos,body1Axis),
		lines[1] = Math_Vec3AddVec3(constraint->body2->absPos,Math_Vec3MultFloat(body1Axis,10));
		World_DrawLinesColor(lines, 2,(Vec4){0,1,1,1});
		lines[0] = Math_Vec3AddVec3(constraint->body2->absPos,coneVector),
		lines[1] = Math_Vec3AddVec3(constraint->body2->absPos,Math_Vec3MultFloat(coneVector,10));
		World_DrawLinesColor(lines, 2,(Vec4){1,0,0,1});
		constraint->J1[0][0] = (Vec3){1,1,1};
		constraint->J1[0][1] = (Vec3){1,1,1};
		constraint->J2[0][0] = (Vec3){0,0,0};
		constraint->J2[0][1] = (Vec3){0,0,0};
		constraint->c1[0] = (Vec3){0,0,0};
		constraint->c1[1] = (Vec3){0,0,0};
		constraint->c2[0] = (Vec3){0,0,0};
		constraint->c2[1] = (Vec3){0,0,0};
	} else {
		Vec3 lines[] = {
		Math_Vec3AddVec3(constraint->body2->absPos,coneAxis),
		Math_Vec3AddVec3(constraint->body2->absPos,Math_Vec3MultFloat(coneAxis,10))};
		World_DrawLinesColor(lines, 2,(Vec4){0,0,1,1});
		lines[0] = Math_Vec3AddVec3(constraint->body2->absPos,body1Axis),
		lines[1] = Math_Vec3AddVec3(constraint->body2->absPos,Math_Vec3MultFloat(body1Axis,10));
		World_DrawLinesColor(lines, 2,(Vec4){1,1,0,1});

		constraint->J1[0][0] = Math_Vec3MultFloat(normal, -1);
		constraint->J1[0][1] = Math_Vec3Cross(point,Math_Vec3MultFloat(normal,1));;
		constraint->J2[0][0] = (Vec3){0,0,0};//Math_Vec3MultFloat(normal,1);
		constraint->J2[0][1] = (Vec3){0,0,0};//Math_Vec3Cross(point,Math_Vec3MultFloat(normal,-1));;
		float dot = Math_Vec3Dot(normal, body1Axis) ;
		constraint->c1[0] = (Vec3){dot,dot,dot};
		constraint->c1[1] = (Vec3){dot,dot,dot};
		constraint->c2[0] = (Vec3){0,0,0};
		constraint->c2[1] = (Vec3){0,0,0};
	}
}

void CreateTrees(PhysicsFigure_t *figure, Bone *bone){
	
	if(bone->nChildren == 0 && !bone->tree){
		FigureTree_t *tree = &figure->trees[figure->nTrees++];
		do {
			tree->bodies[tree->nBodies++] = bone;
			bone->tree = tree;
			bone = bone->parent;
		} while(bone->parent);
	}
	int k;
	for(k=0;k<bone->nChildren; k++){
		CreateTrees(figure, bone->children[k]);
	}
}

void Response(PhysicsFigure_t *figure, PhysicsConstraint_t *constraint){
	FigureTree_t *tree = constraint->body2->tree;

	int j;
	// for(j = 0; j < tree->nBodies; j++){
	for(j = tree->nBodies-1; j >= 0; j--){
		Bone *bone = tree->bodies[j];
		
		// Bone *bone = constraint->body2;
		Vec3 rhs = (Vec3){0,0,0};
		rhs = Math_Vec3MultVec3(constraint->J1[0][0], bone->linVel);

		rhs = Math_Vec3AddVec3(rhs, constraint->c1[0]);

		// bone->linVel = Math_Vec3AddVec3(bone->linVel, rhs);

		rhs = Math_Vec3MultVec3(constraint->J1[0][1], bone->angVel);

		rhs = Math_Vec3AddVec3(rhs, constraint->c1[1]);

		bone->angVel = rhs;//Math_Vec3AddVec3(bone->angVel, rhs);
	}
}


void Physics_ApplyForces(PhysicsFigure_t *figure){	figure->nTrees = 0;
	memset(figure->trees, 0, sizeof(figure->trees));
	int j;
	for(j = 0; j < figure->skel->nBones; j++){
		figure->skel->bones[j].tree = NULL;
	}

	CreateTrees(figure, &figure->skel->bones[0]);
	ConeConstraint_Add(&figure->constraints[0]);
	figure->nConstraints = 1;

	int k;
	for(k = 0; k < figure->nConstraints; k++){
		//figure->constraints[k].Evaluate(&figure->constraints[k]);
		Response(figure, &figure->constraints[k]);
		
	}

}