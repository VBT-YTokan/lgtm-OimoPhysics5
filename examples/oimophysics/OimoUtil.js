/**
 * Defines some shortcuts to creating and adding objects to a world.
 */

import { Quaternion, Vector3 } from '../../build/three.module.js';
import {
    MathUtil,
    Vec3,
    RigidBodyType,
    RigidBodyConfig,
    ShapeConfig,
    RigidBody,
    Shape,
    OConeGeometry,
    OCapsuleGeometry,
    RagdollJointConfig,
    RagdollJoint,
    SphericalJointConfig,
    SphericalJoint,
    SpringDamper,
    RotationalLimitMotor,
    UniversalJointConfig,
    UniversalJoint,
    RevoluteJointConfig,
    PrismaticJoint,
    PrismaticJointConfig,
    RevoluteJoint,
    CylindricalJointConfig,
    CylindricalJoint,
    OBoxGeometry,
    OCylinderGeometry,
    OSphereGeometry, Mat3, Quat
} from './index.js';


export class OimoUtil {
    static addRagdollJoint(w, rb1, rb2, anchor, twistAxis, swingAxis, swingSd = null, maxSwing1Deg = 180, maxSwing2Deg = 180, twistSd = null, twistLm = null) {
        // let invRot1 = rb1.getRotation().transpose();
        // let invRot2 = rb2.getRotation().transpose();
        let jc = new RagdollJointConfig();
        jc.init(rb1, rb2, anchor, twistAxis, swingAxis);
        if (twistSd != null) jc.twistSpringDamper = twistSd;
        if (twistLm != null) jc.twistLimitMotor = twistLm;
        if (swingSd != null) jc.swingSpringDamper = swingSd;
        jc.maxSwingAngle1 = maxSwing1Deg * MathUtil.TO_RADIANS;
        jc.maxSwingAngle2 = maxSwing2Deg * MathUtil.TO_RADIANS;
        let j = new RagdollJoint(jc);
        w.addJoint(j);
        return j;
    }

    static addUniversalJoint(w, rb1, rb2, anchor, axis1, axis2, sd1 = null, lm1 = null, sd2 = null, lm2 = null) {
        // let invRot1 = rb1.getRotation().transpose();
        // let invRot2 = rb2.getRotation().transpose();
        let jc = new UniversalJointConfig();
        jc.init(rb1, rb2, anchor, axis1, axis2);
        if (sd1 != null) jc.springDamper1 = sd1;
        if (lm1 != null) jc.limitMotor1 = lm1;
        if (sd2 != null) jc.springDamper2 = sd2;
        if (lm2 != null) jc.limitMotor2 = lm2;
        let j = new UniversalJoint(jc);
        w.addJoint(j);
        return j;
    }

    static addPrismaticJoint(w, rb1, rb2, anchor, axis, sd = null, lm = null) {
        let jc = new PrismaticJointConfig();
        jc.init(rb1, rb2, anchor, axis);
        if (sd != null) jc.springDamper = sd;
        if (lm != null) jc.limitMotor = lm;
        let j = new PrismaticJoint(jc);
        w.addJoint(j);
        return j;
    }

    static addRevoluteJoint2(w, rb1, rb2, localAnchor1, localAnchor2, localAxis1, localAxis2, sd = null, lm = null) {
        let jc = new RevoluteJointConfig();
        jc.rigidBody1 = rb1;
        jc.rigidBody2 = rb2;

        jc.localAnchor1.copyFrom(localAnchor1);
        jc.localAnchor2.copyFrom(localAnchor2);

        jc.localAxis1.copyFrom(localAxis1);
        jc.localAxis2.copyFrom(localAxis2);

        if (sd != null) jc.springDamper = sd;
        if (lm != null) jc.limitMotor = lm;
        let j = new RevoluteJoint(jc);
        w.addJoint(j);
        return j;
    }

    static addRevoluteJoint(w, rb1, rb2, anchor, axis, sd = null, lm = null) {
        let jc = new RevoluteJointConfig();
        jc.init(rb1, rb2, anchor, axis);
        if (sd != null) jc.springDamper = sd;
        if (lm != null) jc.limitMotor = lm;
        let j = new RevoluteJoint(jc);
        w.addJoint(j);
        return j;
    }

    static addCylindricalJoint(w, rb1, rb2, anchor, axis, rotSd = null, rotLm = null, traSd = null, traLm = null) {
        let jc = new CylindricalJointConfig();
        jc.init(rb1, rb2, anchor, axis);
        if (rotSd != null) jc.rotationalSpringDamper = rotSd;
        if (rotLm != null) jc.rotationalLimitMotor = rotLm;
        if (traSd != null) jc.translationalSpringDamper = traSd;
        if (traLm != null) jc.translationalLimitMotor = traLm;
        let j = new CylindricalJoint(jc);
        w.addJoint(j);
        return j;
    }

    static addSphericalJoint(w, rb1, rb2, anchor) {
        let jc = new SphericalJointConfig();
        jc.init(rb1, rb2, anchor);
        let j = new SphericalJoint(jc);
        w.addJoint(j);
        return j;
    }

    static addSphericalJoint2(w, rb1, rb2, localAnchor1, localAnchor2) {
        let jc = new SphericalJointConfig();
        jc.localAnchor1.copyFrom(localAnchor1);
        jc.localAnchor2.copyFrom(localAnchor2);

        jc.rigidBody1 = rb1;
        jc.rigidBody2 = rb2;

        let j = new SphericalJoint(jc);
        w.addJoint(j);
        return j;
    }

    static addSphere(w, center, radius, wall) {
        return OimoUtil.addRigidBody(w, center, new OSphereGeometry(radius), wall);
    }

    static addBox(w, center, halfExtents, wall) {
        return OimoUtil.addRigidBody(w, center, new OBoxGeometry(halfExtents), wall);
    }

    static addCylinder(w, center, radius, halfHeight, wall) {
        return OimoUtil.addRigidBody(w, center, new OCylinderGeometry(radius, halfHeight), wall);
    }

    static addCone(w, center, radius, halfHeight, wall) {
        return OimoUtil.addRigidBody(w, center, new OConeGeometry(radius, halfHeight), wall);
    }

    static addCapsule(w, center, radius, halfHeight, wall) {
        return OimoUtil.addRigidBody(w, center, new OCapsuleGeometry(radius, halfHeight), wall);
    }

    static addRigidBody(w, center, geom, wall) {
        let shapec = new ShapeConfig();
        shapec.geometry = geom;
        let bodyc = new RigidBodyConfig();
        bodyc.type = wall ? RigidBodyType.STATIC : RigidBodyType.DYNAMIC;
        bodyc.position = center;
        let body = new RigidBody(bodyc);
        body.addShape(new Shape(shapec));
        w.addRigidBody(body);
        return body;
    }

    // ---------------------------------------------------------------------------
    static addRagdoll(w, pos) {
        let head;
        let body1;
        let body2;
        let armL1;
        let armL2;
        let armR1;
        let armR2;
        let legL1;
        let legL2;
        let legR1;
        let legR2;
        let headHeight = 0.3;
        let upperBody = 0.35;
        let lowerBody = 0.35;
        let bodyRadius = 0.2;
        let legRadius = 0.1;
        let legInterval = 0.15;
        let upperLeg = 0.5;
        let lowerLeg = 0.5;
        let armRadius = 0.075;
        let upperArm = 0.35;
        let lowerArm = 0.35;
        head = OimoUtil.addCapsule(w, pos.add(new Vec3(0, lowerBody + upperBody + bodyRadius + headHeight / 2, 0)), headHeight / 2 * 0.8, headHeight / 2 * 0.2, false);
        body1 = OimoUtil.addCapsule(w, pos.add(new Vec3(0, lowerBody + upperBody / 2, 0)), bodyRadius, upperBody / 2, false);
        body2 = OimoUtil.addCapsule(w, pos.add(new Vec3(0, lowerBody / 2, 0)), bodyRadius, lowerBody / 2, false);
        legL1 = OimoUtil.addCapsule(w, pos.add(new Vec3(-legInterval, -upperLeg / 2 - legInterval, 0)), legRadius, upperLeg / 2, false);
        legL2 = OimoUtil.addCapsule(w, pos.add(new Vec3(-legInterval, -upperLeg - lowerLeg / 2 - legInterval, 0)), legRadius, lowerLeg / 2, false);
        legR1 = OimoUtil.addCapsule(w, pos.add(new Vec3(legInterval, -upperLeg / 2 - legInterval, 0)), legRadius, upperLeg / 2, false);
        legR2 = OimoUtil.addCapsule(w, pos.add(new Vec3(legInterval, -upperLeg - lowerLeg / 2 - legInterval, 0)), legRadius, lowerLeg / 2, false);
        armL1 = OimoUtil.addCapsule(w, pos.add(new Vec3(-bodyRadius - upperArm / 2, lowerBody + upperBody, 0)), armRadius, upperArm / 2, false);
        armL2 = OimoUtil.addCapsule(w, pos.add(new Vec3(-bodyRadius - upperArm - lowerArm / 2, lowerBody + upperBody, 0)), armRadius, lowerArm / 2, false);
        armR1 = OimoUtil.addCapsule(w, pos.add(new Vec3(bodyRadius + upperArm / 2, lowerBody + upperBody, 0)), armRadius, upperArm / 2, false);
        armR2 = OimoUtil.addCapsule(w, pos.add(new Vec3(bodyRadius + upperArm + lowerArm / 2, lowerBody + upperBody, 0)), armRadius, lowerArm / 2, false);
        let rotZ90 = new Mat3(null).appendRotationEq(90 * MathUtil.TO_RADIANS, 0, 0, 1);
        armL1.setRotation(rotZ90);
        armL2.setRotation(rotZ90);
        armR1.setRotation(rotZ90);
        armR2.setRotation(rotZ90);
        let x = new Vec3(1, 0, 0);
        let y = new Vec3(0, 1, 0);
        let z = new Vec3(0, 0, 1);
        let sd = new SpringDamper();
        sd.setSpring(10, 1);
        let lm180 = new RotationalLimitMotor().setLimits(-90 * MathUtil.TO_RADIANS, 90 * MathUtil.TO_RADIANS);
        let lm90 = new RotationalLimitMotor().setLimits(-45 * MathUtil.TO_RADIANS, 45 * MathUtil.TO_RADIANS);
        let lmElbow = new RotationalLimitMotor().setLimits(0, 160 * MathUtil.TO_RADIANS);
        let lmKnee = new RotationalLimitMotor().setLimits(0, 160 * MathUtil.TO_RADIANS);
        OimoUtil.addRagdollJoint(w, body1, head, pos.add(new Vec3(0, lowerBody + upperBody + bodyRadius, 0)), y, x, sd, 90, 70, sd, lm180);
        OimoUtil.addRagdollJoint(w, body1, body2, pos.add(new Vec3(0, lowerBody, 0)), y, x, sd, 60, 45, sd, lm90);
        OimoUtil.addRagdollJoint(w, body1, armL1, pos.add(new Vec3(-bodyRadius, lowerBody + upperBody, 0)), x, z, sd, 90, 90, sd, lm180);
        OimoUtil.addRagdollJoint(w, body1, armR1, pos.add(new Vec3(bodyRadius, lowerBody + upperBody, 0)), x.negate(), z, sd, 90, 90, sd, lm180);
        OimoUtil.addRevoluteJoint(w, armL1, armL2, pos.add(new Vec3(-bodyRadius - upperArm, lowerBody + upperBody, 0)), y, sd, lmElbow);
        OimoUtil.addRevoluteJoint(w, armR1, armR2, pos.add(new Vec3(bodyRadius + upperArm, lowerBody + upperBody, 0)), y.negate(), sd, lmElbow);
        let jc = new RagdollJointConfig();
        jc.swingSpringDamper = sd;
        jc.maxSwingAngle1 = 90 * MathUtil.TO_RADIANS;
        jc.maxSwingAngle2 = 70 * MathUtil.TO_RADIANS;
        jc.twistSpringDamper = sd;
        jc.twistLimitMotor = lm180;
        jc.init(body2, legL1, pos.add(new Vec3(-legInterval, -legInterval, 0)), y, x);
        jc.localTwistAxis1 = z.negate();
        w.addJoint(new RagdollJoint(jc));
        jc.init(body2, legR1, pos.add(new Vec3(legInterval, -legInterval, 0)), y, x);
        jc.localTwistAxis1 = z.negate();
        w.addJoint(new RagdollJoint(jc));
        OimoUtil.addRevoluteJoint(w, legL1, legL2, pos.add(new Vec3(-legInterval, -legInterval - upperLeg, 0)), x, sd, lmKnee);
        OimoUtil.addRevoluteJoint(w, legR1, legR2, pos.add(new Vec3(legInterval, -legInterval - upperLeg, 0)), x, sd, lmKnee);

        return body1;
    }



    static vec3FromVector3(position) {
        return new Vec3(...position.toArray());
    }

    static quatFromQuaternion(quaternion) {
        return new Quat(...quaternion.toArray());
    }

    static quatFromEuler(rotation) {
        return new Quat(...new Quaternion().setFromEuler(rotation).toArray());
    }


    static vector3FromVec3(pos) {
        return new Vector3(pos.x, pos.y, pos.z);
    }

    static quaternionFromQuat(quat) {
        return new Quaternion(quat.x, quat.y, quat.z, quat.w);
    }
}
