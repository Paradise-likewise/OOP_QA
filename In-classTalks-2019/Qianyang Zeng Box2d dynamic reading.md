# **Code reading: Dynamic Module of Box2D**

<font size=2>Qianyang Zeng</font>

## Introduction

Dynamic Module** is the key part of BOX2D to build up a new game. 

***Common***, ***collision*** and ***particle*** modules provide the basis, while ***dynamic*** module contains classes and functions as the user interface and undertake the real-time computing tasks, an dynamic module is based on ***Common*** and ***Collision*** module.

When user set up a new game, a virtual world is constructed, and so do new bodies, their attached fixture, contacts and joints. The whole constructing process is carried on by functions in Dynamic module. 

More importantly, dynamic computes and presents the real-time movement and collision. For example, force and torque can be applied to the bodies through contacts and joints, and then the bodies gain velocity and change position, the data are stored in b2Body class. The position and velocity of a body change continuously, sometimes collision happens, so real-time computing process is conducted by “Solve” and “SolveTOI” functions, and the internal classes such as b2Island. Time is divided into many small time-steps (1/60 second for each time-step by default), which is convenient for real-time computing.

The important classes defined in Dynamic module:

**`b2World`:** a virtual world in the game with a fixed value of gravity, in which various items can be added in.

**`b2Body`:** a rigid body created in b2World. 

**`b2Fixture`:** used to attach the shape and other attributes to a b2Body.

**`b2Contact`:** used to contact each two overlapping shapes when the two bodies collides.

**`b2Joint`:** used to constraint two bodies together.

 

## **Classes, data members and functions:**

 

### `class b2World`:

#### **data members:**

`m_blockAllocator`: allocate block memory for the world.

`m_stackAllocator`: allocate stack memory for the world.

`m_flags`: an int32 flag in the world class with three valid bits: the first bit (counting from lower bit) is used to record the newly created fixture and find new contacts in time; second bit is used to record whether the current world is locked, which means the world cannot be modified or dumped; the third bit is used to trigger clearing all the forces in every bodies in the current world, which can be set manually.

`m_contactManager`: store and deal with all the contacts in the world. 

`m_bodyList`: a b2Body pointer stores all the bodies in the current world.

`m_jointList`: a b2Joint pointer stores all the joints between different bodies in the current world.

`m_bodyCount`: record the current number of bodies.

`m_jointCound`: record the current number of joints.

`m_gravity`: b2Vector of gravity.

​         

`m_allowSleep`: another flag of bool. If it is false, all the bodies will be set awake and calculated while collision, which will consume more memory.

`m_destructionListener`: when the body has been destroyed but the attached fixture sand joints have not, it will allow to continue destroying the fixtures and joints.

`m_debugDraw`: used to draw for debug. It must be implemented by user.

`m_inv_dt0`: compute the time step ratio.

`m_warmStarting`, `m_continuousPhysics`, `m_subStepping`: flags for some optimizing operations.

`m_stepComplete`: used to check whether the current time step is over.

`m_profile`: used to store some data while computing the time.

​         

#### **important functions:**

`CreateBody`: add new bodies to the pointer m_bodyList.

`Step`: This function is the key to run the whole world continuously. After checking the flags are correct, it begins to solve out the parameters of the active bodies, such as velocities and positions, by calling the function “Solve”. It also response to the manually changing flags such as checking m_flags to check whether to clear all the forces.

`ClearForces`: clear all the forces in the world.

`DrawDebugData`: draw for debug. It must be implemented by user.

`QueryAABB`: give an AABB and find other overlapping b2AABB from the dynamic tree. The results are recorded in the b2QueryCallback.

`RayCast`: generate a ray to test the fixture. The results are stored in the b2RayCastCallback.

`GetTreeHeight`, `GetTreeBalance`, `GetTreeQuality`: get current message of the dinamic tree.

`ShiftOrigin`: Change the world’s origin.

`Dump`: Dump the world into the log file.

 

`Solve`: protected function called by “step”. It establishes a b2Island class to compute the velocities, positions and joints under the constraints in one time-step, and then clear the island to save memory.

`SolveTOI`: protected function called by “step”. It computes the time of impact and is called after a complete time step has passed. It works also by establishing a b2Island class. It is used to avoid tunnel effect that one object tunnel through another when impacting.

 

### **Internal classes working with `b2World`**: 

**`class b2DestructionListener`**: (introduced in the b2World data members)

**`class b2ContactFilter`**: It is implemented by user to manually decide whether two shapes should collide. 

**`class b2ContactListener`**: It is also implemented by user to get information of current contact, or, callbacks from the contacts. 

**`class b2QueryCallback, b2RayCastCallback`**: used respectively in the functions “QueryAABB” and “RayCast” to report the results.

**`struct b2TimeStep`**: often used to store the time of a single step and number of iterations of velocity (is often is set to 8) and position (is often set to 3), which determine the results of computing the velocity and position in a time step. 

**`class b2Island`**: It plays the key role in computing the velocity and position during a timestep, which is constructed automatically at the beginning of the timestep and cleared at the end of the timestep. It is constructed by b2World’s and other class’s functions “Solve” and “SolveTOI”.

 

### **`class b2Body`**:

#### **data members:**

`m_type`: marks the type of the rigid body. There are three types: 1, static: zero mass, zero velocity, usually set as a stationary object such as ground and wall. It can be moved by user but the velocity is always zero; 2, kinematic: zero mass, can’t be influenced by other dynamic or kinematic bodies, so it don’t collide with others. But user can change its velocity to move it. 3, dynamic: positive mass, non-zero velocity determined by forces.

`m_flags`: an int32 flag used as 7 flags (from the lowest bit): 1, `e_islandFlag`: whether the body is under computing by an island class; 2, `e_awakeFlag`: whether the body is awake so that it can be considered computing by island class; 3, `e_autoSleepFlag`: whether the body can be automatically set to sleep after a period of time (b2_timeToSleep = 0.5f); 4, `e_bulletFlag`: It is set to true when dynamic body has a high speed and may cause tunnel effect. When it is set to true, b2World’s function “Solve” will start the CCD process that the TOI is calculated and the body is added to island class for computing to avoid tunnel effect; 5, `e_fixedRotationFlag`: whether the body is allowed to rotate; 6, `e_activeFlag`: whether the body is available in the broadPhase. If not the fixtures and contacts attached on the body will be removed; 7, `toiFlag`: seems to mean whether the toi of the compact that includes this body has been calculated before?

`m_islandIndex`: index in current island class’s m_bodies array.

 

`m_xf`: the body origin transform.

`m_sweep`: used to define the initial and final transform of the shapes to find whether the shape collides with others. 

`m_linearVelocity`: 

`m_angularVelocity`;

`m_force`;

`m_torque`;

 

`m_world`: b2World that this body belongs to.

`m_prev`: b2Body pointer points to the previous b2Body in the body list of the world.

`m_next`: b2Body pointer points to the next b2Body in the body list of the world.

`m_fixtureList`: contains all the fixtures attached to this body. Note that the shape of this body is stored in attached fixture.

`m_fixtureCount`: number of the fixtures attached to this body.

`m_jointList`: contains all the joints related to this body.

`m_contactList`: contains all the contacts related to this body.

 

`m_mass`, `m_invMass`: mass and its inverse.

`m_I`, `m_invI`: rotational inertia and its inverse.

`m_linearDamping`;

`m_angularDamping`;

`m_gravityScale`: the ratio of the true gravity and the original set gravity. Usually set to 1.

`m_sleepTime`: record the time that the body have sleep

`m_userData`: implemented by user to help user find the body quickly.

#### **important functions:**

`CreateFixture`: Creates a fixture from a shape and attach it to this body.

`DestroyFixture`;

`SetTransform`: Set the position of the body's origin and rotation (set m_xf and initialize m_sweep) and add proxy to the current broadPhase.

`Dump`: Dump this body to a log file.

 

`SynchronizeFixtures`: to synchronize a lager AABB to contain the initial and final shapes

`SynchronizeTransform`: update the m_xf using m_swep?

`ShouldCollide`: whether this body should collide with one another. It considers the body type (should be dynamic) and whether there is a joint preventing collision.

`Advance`: update m_xf and m_sweep by a period of time as parameter (float32 alpha).

 

### **`class b2Fixture`:**

#### **data members:** 

`m_density`: density of this fixture’s parent body.

`m_next`: b2Fixture pointer points to the next fixture in its parent bady’s fixture list.

`m_body`: b2Body pointer points to its parent body.

`m_shape`: b2Shape pointer. What it points to is the body’s shape.

 

`m_friction`: the fiction factor on the shape’s surface. Usually set to be between 0 and 1. 0 means complete smooth.

`m_restitution`: the elastic coefficient of the body. Usually set to be between 0 and 1. 1 means the collision that this body is engaged in is perfectly elastic collision.

 

`m_proxies`: b2FixtureProxy class pointer used to connect the fixture to the broadPhase.

`m_proxyCount`: the number of proxies.

`m_filter`: b2Filter class holding Contact filtering data which is used to manually set its parent body to never collide with some particular bodies.

`m_isSensor`: a bool flag. If set true then after the collision the body will not move by default (island class will skip the calculation of its parent body) and its movement must be implemented by user.

`m_userData`: the same with that in b2Body.

#### **important functions:** 

`SetFilterData`: set m_filter and automatically call the function “Refilter”. 

`Refilter`: set the related Contact’s flag of filtering true so that the Contact containing its parent body will be filtered and skipped when computing contacts.

`TestPoint`: give a point (b2Vec) and test whether the point is inside the shape.

`RayCast`: to reflect the imput ray to the output ray.

 

`Create`: protected class called by its parent body. This fixture is created through this function.

`Destroy`: protected class called by its parent body and world. This fixture is destroyed through this function.

`CreateProxies`: protected class called by its parent body to create proxy holding the parent body’s data on the broadPhase.

`DestroyProxies`: protected class called by its parent body to destroy the proxy on the broadPhase.

`Synchronize`: protected class called by its parent body to synchronize two transforms (one is the initial transform of the body and the other can be the final transform computed by the sweep) to form its new proxy and update its proxy on the broadPhase.

 

### **`class b2Contact`:**

#### **data members:**

`s_registers`: static b2ContactRegister array stored with different kinds of “Create” and “Destroy” functions in different kinds of derived classes from b2Contact, such as circle contact, polygon and circle contact, polygon contact etc. There are 4 kinds of shape in total, so this static array size is 4 by 4.

`s_initialized`: a static bool flag used to tell whether the s_register has been initialized.

 

`m_flag`: int32 flag with 6 valid bits: 1, `e_islandFlag`: used when being calculated by island. If se true, then it is not allowed to be updated; 2, `e_touchingFlag`: set true when the two shapes of this contact’s two parent bodies begin to touch; 3, `e_enabledFlag`: whether this contact is enabled. If it is disabled, then it will be skipped to be calculated by b2Word’s function “Solve” and “SolveTOI”; 4, `e_filterFlag`: set true when its fixture’ m_filter is set true and it will be skipped calculation; 5, `e_bulletHitFlag`: set true when its parent body has a high speed that may cause tunnel effect; 6, `e_toiFlag`: set true when this contact’s m_toi has been calculated but hasn’t been solved. (m_toi remains valid).

 

(`b2ContactEdge` is a class used to connect the body and the contact together. It will be introduced after finishing b2Contact.)

`m_prev`: b2ContactEdge pointer pointing to the previous contact (by pointing to its contact edge) in the world contact list.

`m_next`: b2ContactEdge pointer pointing to the next contact (by pointing to its contact edge) in the world contact list.

`m_nodeA`: (one contact related to two fixture (and its parent body), let’ s call them fixtureA and fixtureB) the b2ContactEdge object for connecting fixtureA (and its parent body).

`m_nodeB`: for connecting fixture (and its parent body).

`m_fixtureA`, `m_fixtureB`: respectively point to fixtureA and fixtureB.

`m_indexA`, `m_indexB`: the fixture’s corresponding proxy’s index in the broadPhase?

`m_manifold`: stored with the detail message of the contact such as the collision point and normal vector.

`m_toiCount`: used to count for the number of TOI cluculating.

`m_toi`: the result of m_toi.

 

`m_friction`: friction factor determined by the two fixtures’ m_friction.

`m_restitution`: elastic coefficient determined by the two fixtures’ m_resrirution.

`m_tangentSpeed`: the normal velocity during or before the collision.

#### **important functions:**

(the following two functions are not the member functions of b2Contact.)

`b2MixFriction`: provide a default way to calculate the friction factor during the two bodies’ contact according to the two bodies’ friction factors: `m_friction = sqrt (fixtureA. m_friction * fixtureB. m_friction)`.

`b2MixRestitution`: provide a default way to calculate the elastic coefficient during the two bodies’ contact according to the two bodies’ elastic coefficients: `m_restitution = max {fixtureA.m_restitution, fixtureB. m_restitution}`.

 

`Evaluate`: to define this contact by user’s manifold and two fixture’s transforms and calculate the process of collision.

`SetFriction`, `SetRestitution`: to not use the default way to calculate m_friction and m_restitution. User set it!

`ResetFriction`, `ResetRestitution`: to reuse the default way to calculate m_friction and m_restitution.

 

***(the following are protected functions)***

`FlagForFiltering`: set m_flag’s e_filterFlag bit true. It is called by its fixture when the fixture is set to be filtered, so that the contact will be skip calculation in world’s functions “Solve” and “SolveTOI”.

`AddType`, `InitializeRegisters`: AddType is called by InitializeRegisters. They are used to initialize the s_register (the static array mentioned before) which sets the different contact create and destroy functions of different types of contacts.

 

`Create`: Called by b2ContactManager to create a contact. This function is the base function of derived class’s “Create” function which stored as function pointers in s_register. It calls the derived class’s “Create” function (by the derived contact class type passed as parameters) which create a derived class of b2Contact and allocate memory for it. 

`Destroy`: Called by b2ContactManager to destroy the contact. It is also the base function of derived class’s “Destroy” function. It calls the derived functions “Destroy”.

`Update`: Called by b2ContactManager (when colliding) and b2World to update this contact’s data, such as m_manifold and m_flag’s e_touchingFlag bit.

 

### **classes working with `b2Contact`:**

**`class b2ContactManager`:** very important class manage all the contacts in b2World and connect the broadPhase with the bodies (the “add” function add fixture’s proxy to m_broadPhase). Its data members: m_broadPhase (stores the broadPhase), m_contactList (stores all the contacts), m_contactCount, m_contactFilter (used to determine whether the corresponding contact should be calculated or skipped while colliding), m_contactListener (test the contact whether the collision is logical, for example, no tunnel effect), m_allocator. 

**`struct b2ContactRegister`:** It is used as a static member array “s_registers” in b2Contact, providing different “Create” and “Destroy” functions of different derived class from b2Contact.

**`struct b2ContactEdge`:** used to connect bodies and contacts together in an imaginary contact graph where each body is a node and each contact is an edge. A contact edge belongs to a doubly linked list maintained in each attached body. Each contact has two contact nodes, one for each attached body.

**`class b2ContactSolver`:** the key class established by island class for calculating the velocities and positions results of the velocity constraint and position constraint.

 

### **derived classes from `b2Contact`:**

**`class b2ChainAndCircleContact`;**

**`class b2ChainAndPolygonContact`;**

**`class b2CircleContact`;**

**`class b2EdgeAndCircleContact`;**

**`class b2EdgeAndPolygonContact`;**

**`class b2PolygonAndCircleContact`;**

**`class b2PolygonContact`;**

**`polymorphic functions used in b2Contact`:**

`Create`, `Destroy`: These two functions are static but not virtual functions, but actually they work as polymorphic functions just like virtual functions, because base class’s (b2Contact’s) Create and Destroy call its derived class’s Create and Destroy through the static array s_register, which works like virtual table. In other words, Box2D has overwrite their own virtual table for polymorphic functions.

`Evaluate`: Virtual function to compute different kind of contacts for different derived class.

 

### **`class b2Joint`:**

#### **data members:**

`m_type`: types of this joint. See more details in the derived classes introduction.

`m_prev`: b2Joint pointer points to the previous joint.

`m_next`: b2Joint pointer points to the next joint.

`m_edgeA`: connect the parent body and this joint together. Each joint is connected with two bodies, so the edgeA is connected with bodyA.

`m_edgeB`: connected with bodyB.

`m_bodyA`, `m_bodyB`: b2Body pointers pointing to the two parent bodies respectively.

 

`m_index`: each joint has its own index in b2World.

`m_islandFlag`: whether this joint is calculated by island class.

`m_collideConnected`: whether the two body can collide. If set false, then the contact manager will cancel the collision between the two bodies and skip its calculation.

`m_userData: the same with the former classes.

#### **important functions:**

`GetAnchorA`, `GetAnchorB`: implemented by the derived classes. The functions find out the anchor points on the two bodies respectively.

`GetReactionForce`, `GetReactionTorque`: get the force and torque respectively on bodyB.

`InitVelocityConstraints`: Compute the constraint for the bodies’ velocity.

`SolveVelocityConstraints`: Solve the constraint.

`SolvePositionConstraints`: Solve the constraint.

**`struct b2JointEdge`:** A joint edge is used to connect bodies and joints together in a joint graph where each body is a node and each joint is an edge. A joint edge belongs to a doubly linked list maintained in each attached body. Each joint has two joint nodes, one for each attached body.

 

### **derived classes from `b2Joint`:**

**`class b2DistanceJoint`:** the distance between the two anchor points is set to be constant.

**`class b2FrictionJoint`:** provide friction between the two bodies.

**`class b2GearJoint`:** connect the two bodies which has already connected with Prismatic Joint or Revolute Joint. The two bodies movement is synchronous as if there is a gear connecting them together.

**`class b2MotorJoint`:** provide torque to rotate the two bodies by the anchor point.

**`class b2MouseJoint`:** use mouse clicking to move the body, like a force from the cursor apply on the anchor point on the body.

**`class b2PrismaticJoint`:** allow the relative movement along a specific axis between the two bodies.

**`class b2PulleyJoint`:** connect the two bodies to a pulley. When one body goes up, the other goes down.

**`class b2RevoluteJoint`:** the two anchor points are the same point which force the two bodies to rotate by this anchor point.

**`class b2RopeJoint`:** restrict the maximum distance between the two anchor points as if there is a rope holding the two points.

**`class b2WeldJoint`:** completely glue the two bodies together (actually the joint is a bit soft since the solver use iteration to calculate the two bodies’ transform).

**`class b2WheelJoint`:** restrict the point on one body to a line on another body.

#### **polymorphic functions used in b2Joint:**

`Create`, `Destroy`: Like b2Contact, the two functions are also static but not virtual, but they also implement that base class function calls the derived class function. It works simply through the joint type stored in “def” as a parameter of the two functions, and use switch to allocate different kinds of derived b2Joint class.

`InitVelocityConstraints`, `SolveVelocityConstraints`, `SolvePositionConstraints`: These three functions are virtual functions. Because different types of joints have different constraints for the bodies, the three functions work differently in different types of derived classes from b2Joint.

`Dump`: dump different message of the derived class into the log file.

`ShiftOrigin`: implemented by user. It can be implemented in every derived class.