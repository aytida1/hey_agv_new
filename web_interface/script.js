class AGVControlCenter {
    constructor() {
        // ROS connection
        this.ros = null;
        this.isConnected = false;
        
        // AGV data
        this.selectedAgv = null;
        this.agvData = {
            agv1: { position: {x: 0, y: 0, z: 0}, orientation: 0, active: false, color: '#e74c3c' },
            agv2: { position: {x: 0, y: 0, z: 0}, orientation: 0, active: false, color: '#2ecc71' },
            agv3: { position: {x: 0, y: 0, z: 0}, orientation: 0, active: false, color: '#f39c12' }
        };
        
        // Initialize smooth movement tracking
        this.agvTargetPositions = {
            agv1: { position: {x: 0, y: 0, z: 0}, orientation: 0 },
            agv2: { position: {x: 0, y: 0, z: 0}, orientation: 0 },
            agv3: { position: {x: 0, y: 0, z: 0}, orientation: 0 }
        };
        
        // Control settings
        this.linearVelocity = 0.5;
        this.angularVelocity = 1.0;
        this.isControlling = false;
        
        // Map data
        this.mapData = null;
        this.mapScale = 20; // pixels per meter
        this.mapOffsetX = 0;
        this.mapOffsetY = 0;
        this.followMode = false;
        
        // Publishers and subscribers
        this.cmdVelPublishers = {};
        this.scanSubscribers = {};
        this.mapSubscriber = null;
        this.tfSubscriber = null;
        
        // Navigation
        this.navGoalPublishers = {};
        this.navActionClients = {};
        this.navigationStatus = 'idle';
        this.currentGoal = null;
        this.goalCompletionInterval = null;
        
        // Predefined locations
        this.locations = {
            home: { x: 0.0, y: 0.0, yaw: 0.0 },
            dock1: { x: 8.657, y: -9.382, yaw: 1.578 },
            dock2: { x: 8.657, y: -11.262, yaw: 1.578 },
            dock3: { x: 8.657, y: -13.142, yaw: 1.578 },
            dock4: { x: 8.657, y: -15.025, yaw: 1.578 },
            center: { x: 0.0, y: -10.0, yaw: 0.0 }
        };
        
        // Laser scan data
        this.laserScanData = {
            agv1: { ranges: [], angle_min: 0, angle_max: 0, angle_increment: 0, range_max: 0, lastUpdate: 0 },
            agv2: { ranges: [], angle_min: 0, angle_max: 0, angle_increment: 0, range_max: 0, lastUpdate: 0 },
            agv3: { ranges: [], angle_min: 0, angle_max: 0, angle_increment: 0, range_max: 0, lastUpdate: 0 }
        };
        
        // UI settings
        this.showLaserScans = true;
        
        // Canvas and rendering
        this.canvas = null;
        this.ctx = null;
        
        // Performance tracking
        this.lastUpdateTime = Date.now();
        this.updateCount = 0;
        this.updateRate = 0;
        
        // Buffer for /all_transforms data
        this.allTransforms = [];
        
        // Smooth movement for AGVs
        this.smoothMovement = true;
        this.movementSpeed = 0.1; // Interpolation speed factor (0.1 = 10% per frame)
        
        this.init();
    }
    
    init() {
        this.setupDOM();
        this.setupCanvas();
        this.connectToROS();
        this.setupKeyboardControls();
        this.setupEventListeners();
        this.startRenderLoop();
    }
    
    setupDOM() {
        // Get DOM elements
        this.elements = {
            connectionStatus: document.getElementById('connectionStatus'),
            agvSelector: document.getElementById('agvSelector'),
            agvStatus: document.getElementById('agvStatus'),
            linearVel: document.getElementById('linearVel'),
            angularVel: document.getElementById('angularVel'),
            linearValue: document.getElementById('linearValue'),
            angularValue: document.getElementById('angularValue'),
            agvPosition: document.getElementById('agvPosition'),
            agvOrientation: document.getElementById('agvOrientation'),
            agvBattery: document.getElementById('agvBattery'),
            agvOperationStatus: document.getElementById('agvOperationStatus'),
            resetView: document.getElementById('resetView'),
            followAgv: document.getElementById('followAgv'),
            zoomIn: document.getElementById('zoomIn'),
            zoomOut: document.getElementById('zoomOut'),
            rosStatus: document.getElementById('rosStatus'),
            lastUpdate: document.getElementById('lastUpdate'),
            updateRate: document.getElementById('updateRate'),
            loadingScreen: document.getElementById('loadingScreen'),
            mapDisplay: document.getElementById('mapDisplay'),
            
            // Navigation elements
            locationBtns: document.querySelectorAll('.location-btn'),
            goalX: document.getElementById('goalX'),
            goalY: document.getElementById('goalY'),
            goalYaw: document.getElementById('goalYaw'),
            sendGoal: document.getElementById('sendGoal'),
            navStatus: document.getElementById('navStatus'),
            navProgress: document.getElementById('navProgress'),
            progressFill: document.getElementById('progressFill'),
            progressText: document.getElementById('progressText'),
            cancelGoal: document.getElementById('cancelGoal')
        };
    }
    
    setupCanvas() {
        this.canvas = this.elements.mapDisplay;
        this.ctx = this.canvas.getContext('2d');
        
        // Set canvas size
        this.resizeCanvas();
        window.addEventListener('resize', () => this.resizeCanvas());
        
        // Mouse events for pan and zoom
        this.setupCanvasEvents();
    }
    
    resizeCanvas() {
        const container = this.canvas.parentElement;
        this.canvas.width = container.clientWidth;
        this.canvas.height = container.clientHeight;
    }
    
    setupCanvasEvents() {
        let isDragging = false;
        let lastMousePos = { x: 0, y: 0 };
        
        this.canvas.addEventListener('mousedown', (e) => {
            isDragging = true;
            lastMousePos = { x: e.clientX, y: e.clientY };
            this.canvas.style.cursor = 'grabbing';
        });
        
        this.canvas.addEventListener('mousemove', (e) => {
            if (isDragging && !this.followMode) {
                const deltaX = e.clientX - lastMousePos.x;
                const deltaY = e.clientY - lastMousePos.y;
                this.mapOffsetX += deltaX;
                this.mapOffsetY += deltaY;
                lastMousePos = { x: e.clientX, y: e.clientY };
            }
            
            // Update mouse position coordinates display
            this.updateMouseCoordinates(e);
        });
        
        this.canvas.addEventListener('mouseup', () => {
            isDragging = false;
            this.canvas.style.cursor = 'move';
        });
        
        this.canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const zoomFactor = e.deltaY > 0 ? 0.9 : 1.1;
            this.mapScale *= zoomFactor;
            this.mapScale = Math.max(5, Math.min(100, this.mapScale));
        });
        
        // Add click event to show coordinates
        this.canvas.addEventListener('click', (e) => {
            this.handleMapClick(e);
        });
    }
    
    connectToROS() {
        console.log('Attempting to connect to ROSBridge...');
        
        // Try multiple connection methods
        const urls = [
            'ws://localhost:9091',
            'ws://127.0.0.1:9091',
            'ws://localhost:9090'  // fallback to default port
        ];
        
        this.tryConnect(urls, 0);
    }
    
    tryConnect(urls, index) {
        if (index >= urls.length) {
            console.error('❌ Failed to connect to any ROSBridge URLs');
            this.updateConnectionStep('websocketStep', 'error');
            this.showRetryButton();
            return;
        }
        
        const url = urls[index];
        console.log(`Trying to connect to: ${url}`);
        
        this.ros = new ROSLIB.Ros({
            url: url
        });
        
        // Set a timeout for connection attempt
        const connectionTimeout = setTimeout(() => {
            console.log(`Connection timeout for ${url}, trying next...`);
            this.ros.close();
            this.tryConnect(urls, index + 1);
        }, 3000);
        
        this.ros.on('connection', () => {
            clearTimeout(connectionTimeout);
            console.log(`✅ Successfully connected to ROSBridge at ${url}`);
            this.isConnected = true;
            this.updateConnectionStatus(true);
            this.updateConnectionStep('websocketStep', 'completed');
            this.updateConnectionStep('topicsStep', 'loading');
            this.setupROSSubscriptions();
            this.setupROSPublishers();
            setTimeout(() => {
                this.updateConnectionStep('topicsStep', 'completed');
                this.hideLoadingScreen();
            }, 1000);
        });
        
        this.ros.on('error', (error) => {
            clearTimeout(connectionTimeout);
            console.error(`❌ Error connecting to ${url}:`, error);
            // Try next URL
            this.tryConnect(urls, index + 1);
        });
        
        this.ros.on('close', () => {
            clearTimeout(connectionTimeout);
            console.log(`⚠️ Connection to ${url} closed`);
            if (this.isConnected) {
                this.isConnected = false;
                this.updateConnectionStatus(false);
                this.attemptReconnect();
            }
        });
    }
    
    setupROSSubscriptions() {
        // Subscribe to map topic - this will show your actual map
        this.mapSubscriber = new ROSLIB.Topic({
            ros: this.ros,
            name: '/map',
            messageType: 'nav_msgs/OccupancyGrid'
        });
        
        this.mapSubscriber.subscribe((message) => {
            console.log('Map data received:', message.info);
            this.handleMapData(message);
        });
        
        // ONLY TF subscriber - this is exactly what tf2_echo uses
        this.tfSubscriber = new ROSLIB.Topic({
            ros: this.ros,
            name: '/tf',
            messageType: 'tf2_msgs/TFMessage'
        });
        
        this.tfSubscriber.subscribe((message) => {
            this.handleTFData(message);
        });
        
        // Also subscribe to /tf_static for static transforms
        this.tfStaticSubscriber = new ROSLIB.Topic({
            ros: this.ros,
            name: '/tf_static',
            messageType: 'tf2_msgs/TFMessage'
        });
        
        this.tfStaticSubscriber.subscribe((message) => {
            this.handleTFData(message);
        });
        
        // Subscribe to laser scans for each AGV
        Object.keys(this.agvData).forEach(agvId => {
            // Laser scan subscription for obstacle detection
            this.scanSubscribers[agvId] = new ROSLIB.Topic({
                ros: this.ros,
                name: `/${agvId}/scan`,
                messageType: 'sensor_msgs/LaserScan'
            });
            
            this.scanSubscribers[agvId].subscribe((message) => {
                this.handleLaserScanData(agvId, message);
            });
            
            // Initialize tracking data
            this.agvData[agvId].lastUpdateTime = Date.now();
        });
        
        // Subscribe to /all_transforms for direct TF buffer
        this.allTransformsSubscriber = new ROSLIB.Topic({
            ros: this.ros,
            name: '/all_transforms',
            messageType: 'tf2_msgs/TFMessage'
        });
        this.allTransformsSubscriber.subscribe((message) => {
            this.handleAllTransforms(message);
        });
        
        console.log('✅ Subscribed to /map, /tf, /tf_static, and laser scans - ONLY using TF data like tf2_echo');
    }
    
    setupROSPublishers() {
        // Setup command velocity publishers for each AGV
        Object.keys(this.agvData).forEach(agvId => {
            this.cmdVelPublishers[agvId] = new ROSLIB.Topic({
                ros: this.ros,
                name: `/${agvId}/cmd_vel`,
                messageType: 'geometry_msgs/Twist'
            });
        });
        
        // Setup initial pose publishers for each AGV
        this.initialPosePublishers = {};
        Object.keys(this.agvData).forEach(agvId => {
            this.initialPosePublishers[agvId] = new ROSLIB.Topic({
                ros: this.ros,
                name: `/${agvId}/initialpose`,
                messageType: 'geometry_msgs/PoseWithCovarianceStamped'
            });
        });
        
        // Setup navigation action clients for each AGV
        Object.keys(this.agvData).forEach(agvId => {
            this.setupNavigationActionClient(agvId);
        });
        
        console.log('✅ Set up publishers for cmd_vel, initialpose topics and navigation action clients');
    }
    
    handleMapData(mapMsg) {
        console.log('✅ Map data received! Resolution:', mapMsg.info.resolution, 'Size:', mapMsg.info.width, 'x', mapMsg.info.height);
        console.log('Map origin:', mapMsg.info.origin.position);
        
        this.mapData = {
            info: mapMsg.info,
            data: mapMsg.data
        };
        
        // Store map info for proper rendering
        this.mapInfo = {
            width: mapMsg.info.width,
            height: mapMsg.info.height,
            resolution: mapMsg.info.resolution, // meters per pixel
            origin: mapMsg.info.origin.position // map origin in world coordinates
        };
        
        console.log(`Map loaded: ${this.mapInfo.width}x${this.mapInfo.height} pixels, ${this.mapInfo.resolution}m/pixel`);
        
        // Auto-adjust view to show the map
        this.resetMapView();
    }
    
    handleOdometryData(agvId, odomMsg) {
        const agv = this.agvData[agvId];
        agv.position = odomMsg.pose.pose.position;
        agv.orientation = this.quaternionToYaw(odomMsg.pose.pose.orientation);
        agv.active = true;
        
        this.updateAGVInfo(agvId);
        this.updatePerformanceStats();
    }
    
    handleAMCLData(agvId, amclMsg) {
        // AMCL provides more accurate localization than odometry
        const agv = this.agvData[agvId];
        agv.position = amclMsg.pose.pose.position;
        agv.orientation = this.quaternionToYaw(amclMsg.pose.pose.orientation);
        agv.active = true;
        
        // Update UI if this is the selected AGV
        if (agvId === this.selectedAgv) {
            this.updateAGVInfo(agvId);
        }
        
        console.log(`AMCL Update - ${agvId}: x=${agv.position.x.toFixed(2)}, y=${agv.position.y.toFixed(2)}, θ=${(agv.orientation * 180 / Math.PI).toFixed(1)}°`);
        this.updatePerformanceStats();
    }
    
    handleTFData(tfMsg) {
        // Store TF transforms for chaining map → odom → base_link
        tfMsg.transforms.forEach(transform => {
            const { header, child_frame_id, transform: tf } = transform;
            const parentFrame = header.frame_id;
            
            // Store transforms in a lookup table for chaining
            if (!this.tfLookup) {
                this.tfLookup = {};
            }
            
            // Store the transform with timestamp for freshness checking
            this.tfLookup[`${parentFrame}->${child_frame_id}`] = {
                transform: tf,
                timestamp: Date.now(),
                header: header
            };
        });
        
        // Now try to compute map → base_link for each AGV by chaining transforms
        Object.keys(this.agvData).forEach(agvId => {
            this.updateAGVPositionFromTF(agvId);
        });
        
        this.updatePerformanceStats();
    }
    
    handleAllTransforms(tfMsg) {
        // Store the full transform buffer as received
        this.allTransforms = tfMsg.transforms;
        // Optionally, update AGV positions for visualization
        this.updateAGVsFromAllTransforms();
    }

    updateAGVsFromAllTransforms() {
        // Use the transforms as-is, no chaining or filtering
        // Find AGV base_link transforms and update target positions for smooth interpolation
        const agvBaseLinks = {
            agv1: 'agv1/base_link',
            agv2: 'agv2/base_link',
            agv3: 'agv3/base_link'
        };
        
        // Reset all AGVs to inactive first
        Object.keys(this.agvData).forEach(agvId => {
            this.agvData[agvId].active = false;
        });
        
        // Update target positions from transform data
        this.allTransforms.forEach(tf => {
            Object.entries(agvBaseLinks).forEach(([agvId, baseLink]) => {
                if (tf.child_frame_id === baseLink) {
                    // Initialize target positions if this AGV is newly detected
                    if (!this.agvData[agvId].active) {
                        this.agvData[agvId].position = {
                            x: tf.transform.translation.x,
                            y: tf.transform.translation.y,
                            z: tf.transform.translation.z
                        };
                        this.agvData[agvId].orientation = this.quaternionToYaw(tf.transform.rotation);
                    }
                    
                    // Ensure target position object exists
                    if (!this.agvTargetPositions[agvId]) {
                        this.agvTargetPositions[agvId] = {
                            position: { x: 0, y: 0, z: 0 },
                            orientation: 0
                        };
                    }
                    
                    // Set target positions for smooth interpolation
                    this.agvTargetPositions[agvId].position = {
                        x: tf.transform.translation.x,
                        y: tf.transform.translation.y,
                        z: tf.transform.translation.z
                    };
                    this.agvTargetPositions[agvId].orientation = this.quaternionToYaw(tf.transform.rotation);
                    this.agvData[agvId].active = true;
                }
            });
        });
        
        // Smoothly interpolate current positions toward target positions
        this.updateSmoothMovement();
    }
    
    updateSmoothMovement() {
        if (!this.smoothMovement) {
            // If smooth movement is disabled, just copy target positions directly
            Object.keys(this.agvData).forEach(agvId => {
                if (this.agvData[agvId].active && this.agvTargetPositions[agvId]) {
                    this.agvData[agvId].position = { ...this.agvTargetPositions[agvId].position };
                    this.agvData[agvId].orientation = this.agvTargetPositions[agvId].orientation;
                }
            });
            return;
        }
        
        Object.keys(this.agvData).forEach(agvId => {
            if (!this.agvData[agvId].active || !this.agvTargetPositions[agvId]) return;
            
            const current = this.agvData[agvId];
            const target = this.agvTargetPositions[agvId];
            
            // Smooth interpolation for position
            current.position.x += (target.position.x - current.position.x) * this.movementSpeed;
            current.position.y += (target.position.y - current.position.y) * this.movementSpeed;
            current.position.z += (target.position.z - current.position.z) * this.movementSpeed;
            
            // Smooth interpolation for orientation (handle angle wrapping)
            let angleDiff = target.orientation - current.orientation;
            
            // Handle angle wrapping (choose shortest path)
            if (angleDiff > Math.PI) {
                angleDiff -= 2 * Math.PI;
            } else if (angleDiff < -Math.PI) {
                angleDiff += 2 * Math.PI;
            }
            
            current.orientation += angleDiff * this.movementSpeed;
            
            // Normalize orientation to [-π, π]
            if (current.orientation > Math.PI) {
                current.orientation -= 2 * Math.PI;
            } else if (current.orientation < -Math.PI) {
                current.orientation += 2 * Math.PI;
            }
        });
    }
    
    updateAGVPositionFromTF(agvId) {
        if (!this.tfLookup) return;
        
        const now = Date.now();
        const maxAge = 1000; // 1 second max age for transforms
        
        // Define the frame names for this AGV
        const mapFrame = 'map';
        const odomFrame = `${agvId}/odom`;
        const baseLinkFrame = `${agvId}/base_link`;
        
        // Get map → odom transform
        const mapToOdomKey = `${mapFrame}->${odomFrame}`;
        const mapToOdom = this.tfLookup[mapToOdomKey];
        
        // Get odom → base_link transform  
        const odomToBaseLinkKey = `${odomFrame}->${baseLinkFrame}`;
        const odomToBaseLink = this.tfLookup[odomToBaseLinkKey];
        
        // Check if we have both transforms and they're fresh
        if (!mapToOdom || !odomToBaseLink) {
            return; // Missing transforms
        }
        
        if (now - mapToOdom.timestamp > maxAge || now - odomToBaseLink.timestamp > maxAge) {
            return; // Stale transforms
        }
        
        // Chain the transformations: map → odom → base_link
        try {
            const mapToBaseLinkTransform = this.chainTransforms(mapToOdom.transform, odomToBaseLink.transform);
            
            // Update AGV position from the chained transform
            this.agvData[agvId].position = {
                x: mapToBaseLinkTransform.translation.x,
                y: mapToBaseLinkTransform.translation.y,
                z: mapToBaseLinkTransform.translation.z
            };
            
            // Convert quaternion to yaw for orientation
            this.agvData[agvId].orientation = this.quaternionToYaw(mapToBaseLinkTransform.rotation);
            this.agvData[agvId].active = true;
            this.agvData[agvId].lastUpdateTime = Date.now();
            
            // Update UI if this is the selected AGV
            if (agvId === this.selectedAgv) {
                this.updateAGVInfo(agvId);
            }
            
            // Throttled logging to reduce console spam
            if (!this.agvData[agvId].lastLogTime || now - this.agvData[agvId].lastLogTime > 1000) {
                console.log(`TF Chain Update - ${agvId}: x=${mapToBaseLinkTransform.translation.x.toFixed(3)}, y=${mapToBaseLinkTransform.translation.y.toFixed(3)}, θ=${(this.agvData[agvId].orientation * 180 / Math.PI).toFixed(1)}°`);
                this.agvData[agvId].lastLogTime = now;
            }
            
        } catch (error) {
            console.warn(`Failed to chain transforms for ${agvId}:`, error);
        }
    }
    
    chainTransforms(transform1, transform2) {
        // Chain two transforms: result = transform1 * transform2
        // This computes the transformation from transform1's parent to transform2's child
        
        // Extract translation and rotation from both transforms
        const t1 = transform1.translation;
        const r1 = transform1.rotation;
        const t2 = transform2.translation;
        const r2 = transform2.rotation;
        
        // Chain translations: t_result = t1 + R1 * t2
        // First rotate t2 by r1, then add t1
        const rotatedT2 = this.rotateVector(t2, r1);
        const resultTranslation = {
            x: t1.x + rotatedT2.x,
            y: t1.y + rotatedT2.y,
            z: t1.z + rotatedT2.z
        };
        
        // Chain rotations: r_result = r1 * r2 (quaternion multiplication)
        const resultRotation = this.multiplyQuaternions(r1, r2);
        
        return {
            translation: resultTranslation,
            rotation: resultRotation
        };
    }
    
    rotateVector(vector, quaternion) {
        // Rotate a vector by a quaternion
        // v' = q * v * q^(-1), where v is treated as a quaternion with w=0
        
        const qx = quaternion.x;
        const qy = quaternion.y;
        const qz = quaternion.z;
        const qw = quaternion.w;
        
        const vx = vector.x;
        const vy = vector.y;
        const vz = vector.z;
        
        // Compute q * v
        const temp_w = -qx * vx - qy * vy - qz * vz;
        const temp_x =  qw * vx + qy * vz - qz * vy;
        const temp_y =  qw * vy + qz * vx - qx * vz;
        const temp_z =  qw * vz + qx * vy - qy * vx;
        
        // Compute (q * v) * q^(-1), where q^(-1) = (-qx, -qy, -qz, qw) for unit quaternions
        return {
            x: temp_w * (-qx) + temp_x * qw + temp_y * (-qz) - temp_z * (-qy),
            y: temp_w * (-qy) + temp_y * qw + temp_z * (-qx) - temp_x * (-qz),
            z: temp_w * (-qz) + temp_z * qw + temp_x * (-qy) - temp_y * (-qx)
        };
    }
    
    multiplyQuaternions(q1, q2) {
        // Multiply two quaternions: result = q1 * q2
        return {
            x: q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
            y: q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
            z: q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
            w: q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        };
    }
    
    quaternionToYaw(quaternion) {
        // Convert quaternion to yaw angle (rotation around Z-axis)
        // Using the formula: yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
        const qw = quaternion.w;
        const qx = quaternion.x;
        const qy = quaternion.y;
        const qz = quaternion.z;
        
        const yaw = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
        return yaw;
    }
    
    yawToQuaternion(yaw) {
        // Convert yaw angle to quaternion (rotation around Z-axis only)
        const halfYaw = yaw * 0.5;
        return {
            x: 0.0,
            y: 0.0,
            z: Math.sin(halfYaw),
            w: Math.cos(halfYaw)
        };
    }
    
    setupKeyboardControls() {
        const activeKeys = new Set();
        
        document.addEventListener('keydown', (e) => {
            // Don't capture keyboard input if user is typing in input fields
            if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') {
                return; // Allow normal input behavior
            }
            
            if (!this.selectedAgv || !this.isConnected) return;
            
            activeKeys.add(e.key.toLowerCase());
            this.handleKeyboardInput(activeKeys);
            e.preventDefault();
        });
        
        document.addEventListener('keyup', (e) => {
            // Don't capture keyboard input if user is typing in input fields
            if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') {
                return; // Allow normal input behavior
            }
            
            activeKeys.delete(e.key.toLowerCase());
            
            if (activeKeys.size === 0) {
                this.stopRobot();
            } else {
                this.handleKeyboardInput(activeKeys);
            }
            e.preventDefault();
        });
        
        // Stop robot when window loses focus
        window.addEventListener('blur', () => {
            this.stopRobot();
            activeKeys.clear();
        });
    }
    
    handleKeyboardInput(activeKeys) {
        let linear = 0;
        let angular = 0;
        
        // Forward/Backward
        if (activeKeys.has('w') || activeKeys.has('arrowup')) {
            linear = this.linearVelocity;
        }
        if (activeKeys.has('s') || activeKeys.has('arrowdown')) {
            linear = -this.linearVelocity;
        }
        
        // Left/Right
        if (activeKeys.has('a') || activeKeys.has('arrowleft')) {
            angular = this.angularVelocity;
        }
        if (activeKeys.has('d') || activeKeys.has('arrowright')) {
            angular = -this.angularVelocity;
        }
        
        // Stop
        if (activeKeys.has(' ')) {
            linear = 0;
            angular = 0;
        }
        
        this.publishVelocity(linear, angular);
    }
    
    publishVelocity(linear, angular) {
        if (!this.selectedAgv || !this.isConnected) return;
        
        const twist = new ROSLIB.Message({
            linear: { x: linear, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: angular }
        });
        
        this.cmdVelPublishers[this.selectedAgv].publish(twist);
        this.isControlling = linear !== 0 || angular !== 0;
        
        // Update operation status
        const status = this.isControlling ? 'Moving' : 'Idle';
        this.elements.agvOperationStatus.textContent = status;
    }
    
    stopRobot() {
        this.publishVelocity(0, 0);
    }
    
    setupEventListeners() {
        // AGV selector
        this.elements.agvSelector.addEventListener('change', (e) => {
            this.selectAGV(e.target.value);
        });
        
        // Velocity controls
        this.elements.linearVel.addEventListener('input', (e) => {
            this.linearVelocity = parseFloat(e.target.value);
            this.elements.linearValue.textContent = this.linearVelocity.toFixed(1);
        });
        
        this.elements.angularVel.addEventListener('input', (e) => {
            this.angularVelocity = parseFloat(e.target.value);
            this.elements.angularValue.textContent = this.angularVelocity.toFixed(1);
        });
        
        // Map controls
        this.elements.resetView.addEventListener('click', () => {
            this.resetMapView();
        });
        
        this.elements.followAgv.addEventListener('click', () => {
            this.toggleFollowMode();
        });
        
        this.elements.zoomIn.addEventListener('click', () => {
            this.mapScale *= 1.2;
            this.mapScale = Math.min(100, this.mapScale);
        });
        
        this.elements.zoomOut.addEventListener('click', () => {
            this.mapScale *= 0.8;
            this.mapScale = Math.max(5, this.mapScale);
        });
        
        // Laser scan toggle
        this.elements.toggleLaser = document.getElementById('toggleLaser');
        this.elements.toggleLaser.addEventListener('click', () => {
            this.toggleLaserScans();
        });
        
        // Smooth movement toggle
        this.elements.toggleSmooth = document.getElementById('toggleSmooth');
        if (this.elements.toggleSmooth) {
            this.elements.toggleSmooth.addEventListener('click', () => {
                this.toggleSmoothMovement();
            });
        }
        
        // Initial Pose controls
        document.getElementById('setInitialPose').addEventListener('click', () => {
            this.setInitialPose();
        });
        
        // Navigation controls
        this.setupNavigationEventListeners();
        
        // Debug navigation button
        document.getElementById('checkNavDebug').addEventListener('click', () => {
            this.checkNavigationInterfaces();
        });
    }
    
    setupNavigationEventListeners() {
        // Location buttons
        this.elements.locationBtns.forEach(btn => {
            btn.addEventListener('click', (e) => {
                const location = e.currentTarget.getAttribute('data-location');
                this.navigateToLocation(location);
            });
        });
        
        // Custom goal button
        this.elements.sendGoal.addEventListener('click', () => {
            this.sendCustomGoal();
        });
        
        // Cancel navigation button
        this.elements.cancelGoal.addEventListener('click', () => {
            this.cancelNavigation();
        });
        
        // Auto-fill goal from map click (future enhancement)
        this.canvas.addEventListener('click', (e) => {
            if (e.shiftKey) { // Hold Shift + Click to set goal
                this.setGoalFromMapClick(e);
            }
        });
    }
    
    selectAGV(agvId) {
        this.selectedAgv = agvId;
        
        if (agvId) {
            this.elements.agvStatus.innerHTML = `<span class="status-active">AGV ${agvId.toUpperCase()} Selected</span>`;
            this.elements.agvStatus.className = 'agv-status';
            this.updateAGVInfo(agvId);
        } else {
            this.elements.agvStatus.innerHTML = `<span class="status-inactive">No AGV Selected</span>`;
            this.elements.agvStatus.className = 'agv-status';
            this.clearAGVInfo();
        }
    }
    
    updateAGVInfo(agvId) {
        if (agvId !== this.selectedAgv) return;
        
        const agv = this.agvData[agvId];
        const pos = agv.position;
        
        this.elements.agvPosition.textContent = `${pos.x.toFixed(2)}, ${pos.y.toFixed(2)}`;
        this.elements.agvOrientation.textContent = `${(agv.orientation * 180 / Math.PI).toFixed(1)}°`;
        this.elements.agvBattery.textContent = '100%'; // Placeholder
        
        if (!this.isControlling) {
            this.elements.agvOperationStatus.textContent = 'Idle';
        }
    }
    
    clearAGVInfo() {
        this.elements.agvPosition.textContent = '-, -';
        this.elements.agvOrientation.textContent = '-';
        this.elements.agvBattery.textContent = '100%';
        this.elements.agvOperationStatus.textContent = 'Idle';
    }
    
    resetMapView() {
        this.mapOffsetX = 0;
        this.mapOffsetY = 0;
        this.mapScale = 20;
        this.followMode = false;
        this.elements.followAgv.classList.remove('btn-primary');
        this.elements.followAgv.classList.add('btn-secondary');
    }
    
    toggleFollowMode() {
        this.followMode = !this.followMode;
        
        if (this.followMode && this.selectedAgv) {
            this.elements.followAgv.classList.remove('btn-secondary');
            this.elements.followAgv.classList.add('btn-primary');
        } else {
            this.elements.followAgv.classList.remove('btn-primary');
            this.elements.followAgv.classList.add('btn-secondary');
        }
    }
    
    toggleLaserScans() {
        this.showLaserScans = !this.showLaserScans;
        
        const button = this.elements.toggleLaser;
        const icon = button.querySelector('i');
        
        if (this.showLaserScans) {
            button.innerHTML = '<i class="fas fa-eye"></i> Hide Laser';
            button.classList.remove('btn-primary');
            button.classList.add('btn-secondary');
        } else {
            button.innerHTML = '<i class="fas fa-eye-slash"></i> Show Laser';
            button.classList.remove('btn-secondary');
            button.classList.add('btn-primary');
        }
        
        console.log(`🔍 Laser scan visualization ${this.showLaserScans ? 'enabled' : 'disabled'}`);
    }
    
    toggleSmoothMovement() {
        this.smoothMovement = !this.smoothMovement;
        
        const button = this.elements.toggleSmooth;
        if (!button) return;
        
        if (this.smoothMovement) {
            button.innerHTML = '<i class="fas fa-water"></i> Smooth: ON';
            button.classList.remove('btn-secondary');
            button.classList.add('btn-primary');
        } else {
            button.innerHTML = '<i class="fas fa-square"></i> Smooth: OFF';
            button.classList.remove('btn-primary');
            button.classList.add('btn-secondary');
            
            // When smooth movement is disabled, immediately snap to target positions
            Object.keys(this.agvData).forEach(agvId => {
                if (this.agvData[agvId].active) {
                    this.agvData[agvId].position = { ...this.agvTargetPositions[agvId].position };
                    this.agvData[agvId].orientation = this.agvTargetPositions[agvId].orientation;
                }
            });
        }
        
        console.log(`🌊 Smooth movement ${this.smoothMovement ? 'enabled' : 'disabled'}`);
    }
    
    startRenderLoop() {
        const render = () => {
            this.renderMap();
            requestAnimationFrame(render);
        };
        render();
    }
    
    renderMap() {
        if (!this.ctx) return;
        const { width, height } = this.canvas;
        this.ctx.clearRect(0, 0, width, height);
        const centerX = width / 2;
        const centerY = height / 2;
        if (this.followMode && this.selectedAgv && this.agvData[this.selectedAgv].active) {
            const agv = this.agvData[this.selectedAgv];
            this.mapOffsetX = centerX - (agv.position.x * this.mapScale);
            this.mapOffsetY = centerY + (agv.position.y * this.mapScale);
        }
        this.ctx.save();
        this.ctx.translate(this.mapOffsetX, this.mapOffsetY);
        if (this.mapData) {
            this.drawOccupancyGrid();
        } else {
            this.drawPlaceholderGrid();
        }
        // Draw AGVs using positions from /all_transforms (as-is)
        this.drawAGVs();
        this.drawLaserScans();
        this.ctx.restore();
        this.drawMapInfo();
    }
    
    drawOccupancyGrid() {
        const { info, data } = this.mapData;
        const { width, height, resolution, origin } = info;
        
        // Create a canvas for the map
        const mapCanvas = document.createElement('canvas');
        mapCanvas.width = width;
        mapCanvas.height = height;
        const mapCtx = mapCanvas.getContext('2d');
        
        const imageData = mapCtx.createImageData(width, height);
        const pixels = imageData.data;
        
        // Convert occupancy grid data to pixel data
        for (let y = 0; y < height; y++) {
            for (let x = 0; x < width; x++) {
                const mapIndex = y * width + x;
                const pixelIndex = mapIndex * 4;
                const value = data[mapIndex];
                
                if (value === -1) {
                    // Unknown space - gray
                    pixels[pixelIndex] = 128;     // R
                    pixels[pixelIndex + 1] = 128; // G
                    pixels[pixelIndex + 2] = 128; // B
                    pixels[pixelIndex + 3] = 255; // A
                } else if (value >= 0 && value < 50) {
                    // Free space - white
                    pixels[pixelIndex] = 255;     // R
                    pixels[pixelIndex + 1] = 255; // G
                    pixels[pixelIndex + 2] = 255; // B
                    pixels[pixelIndex + 3] = 255; // A
                } else {
                    // Occupied space - black
                    pixels[pixelIndex] = 0;       // R
                    pixels[pixelIndex + 1] = 0;   // G
                    pixels[pixelIndex + 2] = 0;   // B
                    pixels[pixelIndex + 3] = 255; // A
                }
            }
        }
        
        mapCtx.putImageData(imageData, 0, 0);
        
        // Draw the map on the main canvas with proper scaling and positioning
        this.ctx.save();
        
        // Convert map coordinates to canvas coordinates
        const mapOriginX = origin.position.x;
        const mapOriginY = origin.position.y;
        
        // Scale to convert from map pixels to world coordinates
        const pixelsPerMeter = 1.0 / resolution;
        const displayScale = this.mapScale / pixelsPerMeter;
        
        // Position the map origin at the correct world coordinates
        this.ctx.translate(mapOriginX * this.mapScale, -mapOriginY * this.mapScale);
        this.ctx.scale(displayScale, -displayScale); // Flip Y axis for ROS coordinate system
        
        this.ctx.drawImage(mapCanvas, 0, 0);
        this.ctx.restore();
    }
    
    drawPlaceholderGrid() {
        // Draw a placeholder grid when no map is available
        this.ctx.strokeStyle = '#ddd';
        this.ctx.lineWidth = 1;
        
        const gridSize = this.mapScale;
        const startX = Math.floor(-this.mapOffsetX / gridSize) * gridSize;
        const startY = Math.floor(-this.mapOffsetY / gridSize) * gridSize;
        
        // Vertical lines
        for (let x = startX; x < this.canvas.width + Math.abs(this.mapOffsetX); x += gridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(x, startY);
            this.ctx.lineTo(x, this.canvas.height + Math.abs(this.mapOffsetY));
            this.ctx.stroke();
        }
        
        // Horizontal lines
        for (let y = startY; y < this.canvas.height + Math.abs(this.mapOffsetY); y += gridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(startX, y);
            this.ctx.lineTo(this.canvas.width + Math.abs(this.mapOffsetX), y);
            this.ctx.stroke();
        }
    }
    
    drawAGVs() {
        // Draw AGVs using the latest positions from /all_transforms (no extra logic)
        Object.entries(this.agvData).forEach(([agvId, agv]) => {
            if (!agv.active) return;
            const x = agv.position.x * this.mapScale;
            const y = -agv.position.y * this.mapScale; // Flip Y axis for ROS coordinate system
            
            this.ctx.save();
            this.ctx.translate(x, y);
            // Use orientation as-is (no offset)
            this.ctx.rotate(-agv.orientation);
            const agvWidth = 0.6 * this.mapScale;  // Increased from 0.3 to 0.6
            const agvHeight = 0.4 * this.mapScale; // Increased from 0.2 to 0.4
            this.ctx.fillStyle = agv.color;
            this.ctx.fillRect(-agvWidth/2, -agvHeight/2, agvWidth, agvHeight);
            this.ctx.fillStyle = '#fff';
            this.ctx.beginPath();
            this.ctx.moveTo(agvWidth/2, 0);
            this.ctx.lineTo(agvWidth/3, -agvHeight/3);
            this.ctx.lineTo(agvWidth/3, agvHeight/3);
            this.ctx.closePath();
            this.ctx.fill();
            this.ctx.restore();
            
            // Simple AGV label
            this.ctx.fillStyle = '#000';
            this.ctx.font = '10px Arial';
            this.ctx.textAlign = 'center';
            this.ctx.fillText(agvId.toUpperCase(), x, y - agvHeight/2 - 5);
        });
    }
    
    drawLaserScans() {
        if (!this.showLaserScans) return; // Allow hiding laser scans
        
        Object.entries(this.laserScanData).forEach(([agvId, scanData]) => {
            if (!this.agvData[agvId].active || scanData.ranges.length === 0) return;
            
            const agv = this.agvData[agvId];
            const agvX = agv.position.x * this.mapScale;
            const agvY = -agv.position.y * this.mapScale; // Flip Y axis
            
            // Laser scan parameters
            const { ranges, angle_min, angle_increment, range_max } = scanData;
            const numPoints = ranges.length;
            
            this.ctx.save();
            this.ctx.translate(agvX, agvY);
            
            // Apply AGV orientation and fix coordinate transformations
            let totalRotation = -agv.orientation; // Base rotation (flip for Y axis)
            
            // Fix for AGV1: Add 180-degree rotation to correct laser orientation
            // if (agvId === 'agv1') {
            //     totalRotation += 2 * Math.PI; // Add 180 degrees
            // }
            
            this.ctx.rotate(totalRotation);
            
            // Draw laser scan points as smaller blue dots for obstacles
            this.ctx.fillStyle = 'rgba(52, 152, 219, 0.9)'; // Changed from red to blue for better visibility
            
            for (let i = 0; i < numPoints; i++) {
                const range = ranges[i];
                
                // Only draw valid range readings (not infinity or invalid)
                if (range > 0.1 && range < range_max && !isNaN(range) && isFinite(range)) {
                    const angle = angle_min + i * angle_increment;
                    
                    // Convert polar coordinates (range, angle) to Cartesian (x, y)
                    const obstacleX = range * Math.cos(angle) * this.mapScale;
                    const obstacleY = -range * Math.sin(angle) * this.mapScale; // Flip Y for display
                    
                    // Draw thinner obstacle point
                    this.ctx.beginPath();
                    this.ctx.arc(obstacleX, obstacleY, 1, 0, 2 * Math.PI); // Reduced from 2 to 1 pixel
                    this.ctx.fill();
                }
            }
            
            // Optional: Draw laser scan range as a faint arc to show sensor coverage
            if (this.selectedAgv === agvId) {
                this.ctx.strokeStyle = 'rgba(52, 152, 219, 0.4)'; // Slightly more visible blue
                this.ctx.lineWidth = 1;
                this.ctx.beginPath();
                
                const maxDisplayRange = Math.min(range_max, 10.0); // Limit display range to 10m
                const arcRadius = maxDisplayRange * this.mapScale;
                
                this.ctx.arc(0, 0, arcRadius, -angle_min, -(angle_min + (numPoints - 1) * angle_increment));
                this.ctx.stroke();
            }
            
            this.ctx.restore();
        });
    }
    
    drawMapInfo() {
        // Draw scale indicator
        this.ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
        this.ctx.fillRect(20, this.canvas.height - 60, 150, 40);
        this.ctx.strokeStyle = '#333';
        this.ctx.strokeRect(20, this.canvas.height - 60, 150, 40);
        
        this.ctx.fillStyle = '#333';
        this.ctx.font = '12px Arial';
        this.ctx.textAlign = 'left';
        this.ctx.fillText(`Scale: ${(this.mapScale/20).toFixed(1)}x`, 30, this.canvas.height - 35);
        this.ctx.fillText(`1m = ${this.mapScale.toFixed(0)}px`, 30, this.canvas.height - 20);
    }
    
    updateConnectionStatus(connected) {
        const status = this.elements.connectionStatus;
        const rosStatus = this.elements.rosStatus;
        
        if (connected) {
            status.className = 'status-connected';
            status.innerHTML = '<i class="fas fa-circle"></i> Connected';
            rosStatus.textContent = 'ROS: Connected';
        } else {
            status.className = 'status-disconnected';
            status.innerHTML = '<i class="fas fa-circle"></i> Disconnected';
            rosStatus.textContent = 'ROS: Disconnected';
        }
    }
    
    updatePerformanceStats() {
        const now = Date.now();
        this.updateCount++;
        
        if (now - this.lastUpdateTime >= 1000) {
            this.updateRate = this.updateCount;
            this.updateCount = 0;
            this.lastUpdateTime = now;
            
            this.elements.updateRate.textContent = `Rate: ${this.updateRate} Hz`;
            this.elements.lastUpdate.textContent = `Last Update: ${new Date().toLocaleTimeString()}`;
        }
    }
    
    updateConnectionStep(stepId, status) {
        const step = document.getElementById(stepId);
        if (!step) return;
        
        const icon = step.querySelector('i');
        step.classList.remove('completed', 'error');
        
        if (status === 'completed') {
            step.classList.add('completed');
            icon.className = 'fas fa-check-circle';
        } else if (status === 'error') {
            step.classList.add('error');
            icon.className = 'fas fa-times-circle';
        } else if (status === 'loading') {
            icon.className = 'fas fa-circle-notch fa-spin';
        }
    }
    
    showRetryButton() {
        const retryBtn = document.getElementById('retryConnection');
        if (retryBtn) {
            retryBtn.style.display = 'block';
            retryBtn.onclick = () => {
                retryBtn.style.display = 'none';
                this.resetConnectionSteps();
                this.connectToROS();
            };
        }
    }
    
    resetConnectionSteps() {
        this.updateConnectionStep('rosbridgeStep', 'loading');
        this.updateConnectionStep('websocketStep', 'loading');
        this.updateConnectionStep('topicsStep', 'loading');
    }

    hideLoadingScreen() {
        this.elements.loadingScreen.classList.add('hidden');
    }
    
    showReconnectMessage() {
        console.log('Attempting to reconnect...');
    }
    
    attemptReconnect() {
        setTimeout(() => {
            if (!this.isConnected) {
                console.log('Attempting to reconnect to ROS...');
                this.resetConnectionSteps();
                this.connectToROS();
            }
        }, 5000);
    }

    setInitialPose() {
        if (!this.selectedAgv || !this.isConnected) {
            this.showPoseStatus('Please select an AGV and ensure ROS connection', false);
            return;
        }
        
        // Get values from input fields
        const x = parseFloat(document.getElementById('initialX').value) || 0.0;
        const y = parseFloat(document.getElementById('initialY').value) || 0.0;
        const yawDegrees = parseFloat(document.getElementById('initialYaw').value) || 0.0;
        
        // Convert degrees to radians
        const yawRadians = yawDegrees * Math.PI / 180.0;
        
        // Create a PoseWithCovarianceStamped message (same format as initial_pose_setter.py)
        const poseMsg = new ROSLIB.Message({
            header: {
                frame_id: 'map',
                stamp: {
                    sec: Math.floor(Date.now() / 1000),
                    nanosec: (Date.now() % 1000) * 1000000
                }
            },
            pose: {
                pose: {
                    position: {
                        x: x,
                        y: y,
                        z: 0.0
                    },
                    orientation: this.yawToQuaternion(yawRadians)
                },
                // Covariance matrix (same as in initial_pose_setter.py)
                covariance: [
                    0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.068
                ]
            }
        });
        
        // Publish the initial pose to the selected AGV
        this.initialPosePublishers[this.selectedAgv].publish(poseMsg);
        
        // Update the AGV's position in our local data for immediate visual feedback
        this.agvData[this.selectedAgv].position.x = x;
        this.agvData[this.selectedAgv].position.y = y;
        this.agvData[this.selectedAgv].orientation = yawRadians;
        this.agvData[this.selectedAgv].active = true;
        
        // Update the AGV info display
        this.updateAGVInfo(this.selectedAgv);
        
        // Show success message
        this.showPoseStatus(`Initial pose set for ${this.selectedAgv.toUpperCase()}: (${x}, ${y}, ${yawDegrees}°)`, true);
        
        console.log(`📍 Initial pose set for ${this.selectedAgv}:`, {
            position: { x, y, z: 0 },
            yaw: `${yawDegrees}° (${yawRadians.toFixed(3)} rad)`
        });
    }
    
    showPoseStatus(message, success = true) {
        const statusDiv = document.getElementById('poseStatus');
        const statusSpan = statusDiv.querySelector('span');
        
        statusSpan.textContent = message;
        statusSpan.className = success ? 'status-success' : 'status-error';
        statusDiv.style.display = 'block';
        
        // Auto-hide after 3 seconds
        setTimeout(() => {
            statusDiv.style.display = 'none';
        }, 3000);
    }
    
    handleLaserScanData(agvId, scanMsg) {
        const now = Date.now();
        const scanData = this.laserScanData[agvId];
        
        // Store laser scan parameters
        scanData.ranges = scanMsg.ranges;
        scanData.angle_min = scanMsg.angle_min;
        scanData.angle_max = scanMsg.angle_max;
        scanData.angle_increment = scanMsg.angle_increment;
        scanData.range_max = scanMsg.range_max;
        scanData.lastUpdate = now;
        
        // Log scan data reception (throttled)
        if (now - scanData.lastLogTime > 2000) {
            console.log(`🔍 Laser scan received for ${agvId}: ${scanData.ranges.length} points, range: ${scanData.angle_min.toFixed(2)} to ${scanData.angle_max.toFixed(2)} rad`);
            scanData.lastLogTime = now;
        }
    }
    
    handleMapClick(e) {
        // Get canvas coordinates
        const rect = this.canvas.getBoundingClientRect();
        const canvasX = e.clientX - rect.left;
        const canvasY = e.clientY - rect.top;
        
        // Convert canvas coordinates to world coordinates
        const worldCoords = this.canvasToWorldCoordinates(canvasX, canvasY);
        
        // Show coordinates in a popup or overlay
        this.showCoordinatePopup(worldCoords.x, worldCoords.y, canvasX, canvasY);
        
        console.log(`🎯 Map clicked at: World(${worldCoords.x.toFixed(2)}, ${worldCoords.y.toFixed(2)}) Canvas(${canvasX}, ${canvasY})`);
    }
    
    updateMouseCoordinates(e) {
        // Get canvas coordinates
        const rect = this.canvas.getBoundingClientRect();
        const canvasX = e.clientX - rect.left;
        const canvasY = e.clientY - rect.top;
        
        // Convert canvas coordinates to world coordinates
        const worldCoords = this.canvasToWorldCoordinates(canvasX, canvasY);
        
        // Update coordinates display in the UI
        this.updateCoordinateDisplay(worldCoords.x, worldCoords.y);
    }
    
    canvasToWorldCoordinates(canvasX, canvasY) {
        // Convert canvas pixel coordinates to world coordinates (meters)
        const worldX = (canvasX - this.mapOffsetX) / this.mapScale;
        const worldY = -(canvasY - this.mapOffsetY) / this.mapScale; // Flip Y axis for ROS coordinates
        
        return { x: worldX, y: worldY };
    }
    
    showCoordinatePopup(x, y, canvasX, canvasY) {
        // Remove existing popup if any
        const existingPopup = document.getElementById('coordinatePopup');
        if (existingPopup) {
            existingPopup.remove();
        }
        
        // Create popup element
        const popup = document.createElement('div');
        popup.id = 'coordinatePopup';
        popup.className = 'coordinate-popup';
        popup.innerHTML = `
            <div class="popup-content">
                <div class="popup-header">
                    <i class="fas fa-map-marker-alt"></i>
                    <span>Map Coordinates</span>
                    <button class="popup-close" onclick="this.parentElement.parentElement.parentElement.remove()">
                        <i class="fas fa-times"></i>
                    </button>
                </div>
                <div class="popup-body">
                    <div class="coord-row">
                        <span class="coord-label">X:</span>
                        <span class="coord-value">${x.toFixed(3)} m</span>
                        <button class="copy-btn" onclick="navigator.clipboard.writeText('${x.toFixed(3)}')">
                            <i class="fas fa-copy"></i>
                        </button>
                    </div>
                    <div class="coord-row">
                        <span class="coord-label">Y:</span>
                        <span class="coord-value">${y.toFixed(3)} m</span>
                        <button class="copy-btn" onclick="navigator.clipboard.writeText('${y.toFixed(3)}')">
                            <i class="fas fa-copy"></i>
                        </button>
                    </div>
                    <div class="popup-actions">
                        <button class="btn btn-primary btn-small" onclick="window.agvControl.copyCoordinates(${x}, ${y})">
                            <i class="fas fa-copy"></i> Copy Both
                        </button>
                        <button class="btn btn-secondary btn-small" onclick="window.agvControl.setCoordinatesInForm(${x}, ${y})">
                            <i class="fas fa-crosshairs"></i> Use for Initial Pose
                        </button>
                    </div>
                </div>
            </div>
        `;
        
        // Position popup near click location
        popup.style.position = 'absolute';
        popup.style.left = (canvasX + 10) + 'px';
        popup.style.top = (canvasY + 10) + 'px';
        popup.style.zIndex = '1000';
        
        // Add to canvas container
        this.canvas.parentElement.appendChild(popup);
        
        // Auto-remove after 5 seconds
        setTimeout(() => {
            if (popup.parentElement) {
                popup.remove();
            }
        }, 5000);
    }
    
    copyCoordinates(x, y) {
        const coordText = `${x.toFixed(3)}, ${y.toFixed(3)}`;
        navigator.clipboard.writeText(coordText).then(() => {
            console.log(`📋 Copied coordinates: ${coordText}`);
            // Show brief feedback
            this.showCoordinatesCopied();
        });
    }
    
    setCoordinatesInForm(x, y) {
        // Fill the initial pose input fields with the clicked coordinates
        document.getElementById('initialX').value = x.toFixed(2);
        document.getElementById('initialY').value = y.toFixed(2);
        
        // Remove the popup
        const popup = document.getElementById('coordinatePopup');
        if (popup) popup.remove();
        
        console.log(`📍 Set initial pose form to: (${x.toFixed(2)}, ${y.toFixed(2)})`);
    }
    
    showCoordinatesCopied() {
        // Show a brief "Copied!" message
        const message = document.createElement('div');
        message.className = 'copied-message';
        message.innerHTML = '<i class="fas fa-check"></i> Coordinates Copied!';
        message.style.position = 'fixed';
        message.style.top = '20px';
        message.style.right = '20px';
        message.style.zIndex = '9999';
        
        document.body.appendChild(message);
        
        setTimeout(() => {
            message.remove();
        }, 2000);
    }
    
    updateCoordinateDisplay(x, y) {
        // Update a persistent coordinate display (we'll add this to the UI)
        let coordDisplay = document.getElementById('mouseCoordinates');
        if (!coordDisplay) {
            coordDisplay = document.createElement('div');
            coordDisplay.id = 'mouseCoordinates';
            coordDisplay.className = 'mouse-coordinates';
            this.canvas.parentElement.appendChild(coordDisplay);
        }
        
        coordDisplay.innerHTML = `<i class="fas fa-crosshairs"></i> ${x.toFixed(2)}, ${y.toFixed(2)}`;
    }
    
    // ==================== NAVIGATION METHODS ====================
    
    navigateToLocation(locationName) {
        if (!this.selectedAgv || !this.isConnected) {
            this.showNavigationStatus('Please select an AGV and ensure ROS connection', 'error');
            return;
        }
        
        const location = this.locations[locationName];
        if (!location) {
            this.showNavigationStatus(`Unknown location: ${locationName}`, 'error');
            return;
        }
        
        console.log(`🧭 Navigating ${this.selectedAgv.toUpperCase()} to ${locationName}: (${location.x}, ${location.y}, ${location.yaw * 180 / Math.PI}°)`);
        
        this.sendNavigationGoal(location.x, location.y, location.yaw, locationName);
    }
    
    sendCustomGoal() {
        if (!this.selectedAgv || !this.isConnected) {
            this.showNavigationStatus('Please select an AGV and ensure ROS connection', 'error');
            return;
        }
        
        // Get custom goal coordinates from input fields
        const x = parseFloat(this.elements.goalX.value);
        const y = parseFloat(this.elements.goalY.value);
        const yawDegrees = parseFloat(this.elements.goalYaw.value) || 0.0;
        
        // Validate inputs
        if (isNaN(x) || isNaN(y)) {
            this.showNavigationStatus('Please enter valid X and Y coordinates', 'error');
            return;
        }
        
        const yawRadians = yawDegrees * Math.PI / 180.0;
        
        console.log(`🎯 Sending custom goal to ${this.selectedAgv.toUpperCase()}: (${x}, ${y}, ${yawDegrees}°)`);
        
        this.sendNavigationGoal(x, y, yawRadians, 'Custom Goal');
    }
    
    // Simplified navigation goal sending using goal_pose topic only
    sendNavigationGoal(x, y, yaw, goalName = 'Goal') {
        console.log(`🎯 Starting navigation to ${goalName} at (${x.toFixed(2)}, ${y.toFixed(2)}, ${(yaw * 180 / Math.PI).toFixed(1)}°)`);
        console.log(`🤖 Selected AGV: ${this.selectedAgv}`);
        
        if (!this.selectedAgv) {
            console.error('❌ No AGV selected');
            alert('Please select an AGV first');
            return;
        }
        
        // Use direct goal_pose topic publishing
        this.sendNavigationGoalViaTopic(x, y, yaw, goalName);
        
        // Start checking for goal completion
        this.startGoalCompletionChecker();
    }
    
    sendNavigationGoalInternal(x, y, yaw, goalName = 'Goal') {
        // Create NavigateToPose goal message (fixed structure)
        const goal = new ROSLIB.Message({
            pose: {
                header: {
                    frame_id: 'map',
                    stamp: {
                        sec: Math.floor(Date.now() / 1000),
                        nanosec: (Date.now() % 1000) * 1000000
                    }
                },
                pose: {
                    position: {
                        x: x,
                        y: y,
                        z: 0.0
                    },
                    orientation: this.yawToQuaternion(yaw)
                }
            },
            behavior_tree: '' // Use default behavior tree
        });
        
        // Store current goal info
        this.currentGoal = {
            x: x,
            y: y,
            yaw: yaw,
            name: goalName,
            agv: this.selectedAgv,
            startTime: Date.now()
        };
        
        // Update UI
        this.navigationStatus = 'active';
        this.showNavigationStatus(`Navigating to ${goalName}...`, 'active');
        this.updateNavigationProgress(0);
        this.elements.cancelGoal.style.display = 'inline-block';
        
        // Check if action client is ready
        console.log('🔍 Action client status:', this.navActionClients[this.selectedAgv]);
        
        // Send goal
        const actionGoal = new ROSLIB.Goal({
            actionClient: this.navActionClients[this.selectedAgv],
            goalMessage: goal
        });
        
        // Add comprehensive debugging
        actionGoal.on('status', (status) => {
            console.log('🔄 Navigation goal status:', status);
        });
        
        actionGoal.on('feedback', (feedback) => {
            console.log('📡 Navigation feedback received:', feedback);
            this.handleNavigationFeedback(feedback);
        });
        
        actionGoal.on('result', (result) => {
            console.log('🏁 Navigation result received:', result);
            this.handleNavigationResult(result);
        });
        
        actionGoal.on('timeout', () => {
            console.log('🕐 Navigation goal timed out');
            this.showNavigationStatus('Navigation goal timed out', 'error');
            this.navigationStatus = 'idle';
            this.currentActionGoal = null;
            this.elements.cancelGoal.style.display = 'none';
        });
        
        actionGoal.on('error', (error) => {
            console.error('❌ Navigation goal error:', error);
            this.showNavigationStatus(`Navigation error: ${error}`, 'error');
            this.navigationStatus = 'idle';
            this.currentActionGoal = null;
            this.elements.cancelGoal.style.display = 'none';
        });
        
        // Add sent event handler
        actionGoal.on('sent', () => {
            console.log('📤 Navigation goal successfully sent to action server');
        });
        
        console.log('🚀 Sending navigation goal...');
        actionGoal.send();
        console.log('✅ Navigation goal send() called successfully');
        
        // Add a timeout check
        setTimeout(() => {
            if (this.navigationStatus === 'active' && this.currentActionGoal === actionGoal) {
                console.log('⚠️ No response from action server after 1 seconds');
                console.log('🔍 Action client connected:', this.navActionClients[this.selectedAgv].isConnected);
            }
        }, 1000);
        
        // Store action goal for cancellation
        this.currentActionGoal = actionGoal;
        
        console.log(`📤 Navigation goal sent to ${this.selectedAgv}: ${goalName} at (${x.toFixed(2)}, ${y.toFixed(2)}, ${(yaw * 180 / Math.PI).toFixed(1)}°)`);
        console.log('🔍 Goal message:', JSON.stringify(goal, null, 2));
    }
    
    sendNavigationGoalViaTopic(x, y, yaw, goalName = 'Goal') {
        console.log('🔄 Using fallback method: direct topic publishing');
        
        // Create geometry_msgs/PoseStamped message for /move_base_simple/goal topic
        const poseStamped = new ROSLIB.Message({
            header: {
                frame_id: 'map',
                stamp: {
                    sec: Math.floor(Date.now() / 1000),
                    nanosec: (Date.now() % 1000) * 1000000
                }
            },
            pose: {
                position: {
                    x: x,
                    y: y,
                    z: 0.0
                },
                orientation: this.yawToQuaternion(yaw)
            }
        });
        
        // Create publisher for simple goal topic
        const goalTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: `/${this.selectedAgv}/goal_pose`,
            messageType: 'geometry_msgs/PoseStamped'
        });
        
        // Publish the goal
        console.log('📤 Publishing navigation goal to topic:', `/${this.selectedAgv}/goal_pose`);
        goalTopic.publish(poseStamped);
        
        // Store current goal info
        this.currentGoal = {
            x: x,
            y: y,
            yaw: yaw,
            name: goalName,
            agv: this.selectedAgv,
            startTime: Date.now()
        };
        
        // Update UI
        this.navigationStatus = 'active';
        this.showNavigationStatus(`Navigating to ${goalName} (via topic)...`, 'active');
        this.updateNavigationProgress(0);
        this.elements.cancelGoal.style.display = 'inline-block';
        
        console.log(`📤 Navigation goal sent via topic to ${this.selectedAgv}: ${goalName} at (${x.toFixed(2)}, ${y.toFixed(2)}, ${(yaw * 180 / Math.PI).toFixed(1)}°)`);
    }
    
    startGoalCompletionChecker() {
        // Clear any existing checker
        if (this.goalCompletionInterval) {
            clearInterval(this.goalCompletionInterval);
        }
        
        // Check goal completion every 100ms for smooth progress updates
        this.goalCompletionInterval = setInterval(() => {
            this.checkGoalCompletion();
        }, 100);
        
        console.log('🔄 Started goal completion checker');
    }
    
    checkGoalCompletion() {
        if (!this.currentGoal || !this.selectedAgv || !this.agvData[this.selectedAgv].active) {
            return;
        }
        
        const agv = this.agvData[this.selectedAgv];
        const goalX = this.currentGoal.x;
        const goalY = this.currentGoal.y;
        const goalYaw = this.currentGoal.yaw;
        const currentX = agv.position.x;
        const currentY = agv.position.y;
        const currentYaw = agv.orientation;
        
        // Calculate distance to goal
        const distanceToGoal = Math.sqrt(
            Math.pow(goalX - currentX, 2) + Math.pow(goalY - currentY, 2)
        );
        
        // Calculate yaw difference (handle wrap-around)
        let yawDiff = Math.abs(goalYaw - currentYaw);
        if (yawDiff > Math.PI) {
            yawDiff = 2 * Math.PI - yawDiff;
        }
        const yawDiffDegrees = yawDiff * 180 / Math.PI;
        
        // Store initial distance for progress calculation
        if (!this.currentGoal.initialDistance) {
            this.currentGoal.initialDistance = Math.max(distanceToGoal, 0.1); // Minimum 0.1m to avoid division by zero
        }
        
        // Calculate progress percentage
        const progress = Math.max(0, Math.min(100, 
            (1 - distanceToGoal / this.currentGoal.initialDistance) * 100
        ));
        
        this.updateNavigationProgress(progress);
        
        // Update status with distance info
        this.showNavigationStatus(
            `Navigating to ${this.currentGoal.name}... (${distanceToGoal.toFixed(1)}m, ${yawDiffDegrees.toFixed(1)}° remaining)`,
            'active'
        );
        
        // Check if goal is reached (0.05m position tolerance, 1 degree yaw tolerance)
        const positionTolerance = 0.05; // 5cm
        const yawToleranceDegrees = 1.0;  // 1 degree
        
        if (distanceToGoal <= positionTolerance && yawDiffDegrees <= yawToleranceDegrees) {
            console.log(`🎯 Goal reached! Distance: ${distanceToGoal.toFixed(3)}m, Yaw diff: ${yawDiffDegrees.toFixed(1)}°`);
            this.handleGoalReached();
        }
    }
    
    handleGoalReached() {
        // Clear the completion checker
        if (this.goalCompletionInterval) {
            clearInterval(this.goalCompletionInterval);
            this.goalCompletionInterval = null;
        }
        
        // Reset navigation state
        this.navigationStatus = 'idle';
        this.elements.cancelGoal.style.display = 'none';
        
        // Show success message
        this.showNavigationStatus(
            `✅ Successfully reached ${this.currentGoal.name}!`,
            'success'
        );
        this.updateNavigationProgress(100);
        
        // Auto-hide success message after 3 seconds
        setTimeout(() => {
            this.hideNavigationStatus();
        }, 3000);
        
        // Clear current goal
        this.currentGoal = null;
        
        console.log('🏁 Navigation completed successfully');
    }
    
    setupNavigationActionClient(agvId) {
        // Simplified setup - no longer using action clients, kept for compatibility
        console.log(`🔧 Navigation setup for ${agvId} (using topic-based approach)`);
    }
    
    handleNavigationFeedback(feedback) {
        // Calculate progress based on distance to goal
        if (this.currentGoal && this.selectedAgv && this.agvData[this.selectedAgv].active) {
            const agv = this.agvData[this.selectedAgv];
            const goalX = this.currentGoal.x;
            const goalY = this.currentGoal.y;
            const currentX = agv.position.x;
            const currentY = agv.position.y;
            
            // Calculate distance to goal
            const distanceToGoal = Math.sqrt(
                Math.pow(goalX - currentX, 2) + Math.pow(goalY - currentY, 2)
            );
            
            // Estimate initial distance (stored when goal was sent)
            if (!this.currentGoal.initialDistance) {
                this.currentGoal.initialDistance = distanceToGoal;
            }
            
            // Calculate progress percentage
            const progress = Math.max(0, Math.min(100, 
                (1 - distanceToGoal / this.currentGoal.initialDistance) * 100
            ));
            
            this.updateNavigationProgress(progress);
            
            // Update status with distance info
            this.showNavigationStatus(
                `Navigating to ${this.currentGoal.name}... (${distanceToGoal.toFixed(1)}m remaining)`,
                'active'
            );
        }
        
        console.log('📍 Navigation feedback:', feedback);
    }
    
    handleNavigationResult(result) {
        console.log('🏁 Navigation result:', result);
        
        // Reset navigation state
        this.navigationStatus = 'idle';
        this.currentActionGoal = null;
        this.elements.cancelGoal.style.display = 'none';
        
        // Check result and show appropriate message
        if (result && result.error_code !== undefined) {
            if (result.error_code === 0) {
                // Success (nav2_msgs/NavigateToPose result codes: 0 = NONE/success)
                this.showNavigationStatus(
                    `✅ Successfully reached ${this.currentGoal.name}!`,
                    'success'
                );
                this.updateNavigationProgress(100);
                
                // Auto-hide success message after 3 seconds
                setTimeout(() => {
                    this.hideNavigationStatus();
                }, 3000);
            } else {
                // Failure with error code
                const errorMsg = result.error_msg || `Error code: ${result.error_code}`;
                this.showNavigationStatus(
                    `❌ Navigation to ${this.currentGoal.name} failed: ${errorMsg}`,
                    'error'
                );
                this.updateNavigationProgress(0);
                
                // Auto-hide error message after 5 seconds
                setTimeout(() => {
                    this.hideNavigationStatus();
                }, 5000);
            }
        } else {
            // Unknown result format
            this.showNavigationStatus(
                `❌ Navigation to ${this.currentGoal.name} failed (unknown result)`,
                'error'
            );
            this.updateNavigationProgress(0);
            
            console.log('Navigation result:', result);
            
            // Auto-hide error message after 5 seconds
            setTimeout(() => {
                this.hideNavigationStatus();
            }, 5000);
        }
        
        // Clear current goal
        this.currentGoal = null;
    }
    
    cancelNavigation() {
        console.log('🛑 Cancelling navigation...');
        
        // Clear goal completion checker
        if (this.goalCompletionInterval) {
            clearInterval(this.goalCompletionInterval);
            this.goalCompletionInterval = null;
        }
        
        // Cancel action goal if exists
        if (this.currentActionGoal) {
            this.currentActionGoal.cancel();
            this.currentActionGoal = null;
        }
        
        // Update UI
        this.navigationStatus = 'idle';
        this.showNavigationStatus('Navigation cancelled', 'warning');
        this.updateNavigationProgress(0);
        this.elements.cancelGoal.style.display = 'none';
        
        // Clear current goal
        this.currentGoal = null;
        
        // Auto-hide message after 3 seconds
        setTimeout(() => {
            this.hideNavigationStatus();
        }, 3000);
    }
    
    showNavigationStatus(message, type = 'info') {
        this.elements.navStatus.textContent = message;
        this.elements.navStatus.className = `nav-status ${type}`;
        this.elements.navStatus.style.display = 'block';
        
        console.log(`🧭 Nav Status [${type.toUpperCase()}]: ${message}`);
    }
    
    hideNavigationStatus() {
        // Reset to default "Ready" state when navigation completes
        this.elements.navStatus.textContent = 'Ready';
        this.elements.navStatus.className = 'nav-status status-idle';
        this.elements.navStatus.style.display = 'block';
    }
    
    updateNavigationProgress(percentage) {
        const progress = Math.max(0, Math.min(100, percentage));
        
        this.elements.progressFill.style.width = `${progress}%`;
        this.elements.progressText.textContent = `${progress.toFixed(0)}%`;
        
        // Show/hide progress bar based on navigation status
        if (this.navigationStatus === 'active' && progress > 0) {
            this.elements.navProgress.style.display = 'block';
        } else if (this.navigationStatus === 'idle') {
            // When navigation is idle, hide progress bar immediately for better UX
            if (progress >= 100) {
                // Keep visible briefly for success/completion, then hide
                setTimeout(() => {
                    this.elements.navProgress.style.display = 'none';
                }, 2000);
            } else {
                // Hide immediately for cancelled or failed navigation
                this.elements.navProgress.style.display = 'none';
            }
        }
    }
    
    setGoalFromMapClick(e) {
        // Convert click coordinates to world coordinates
        const rect = this.canvas.getBoundingClientRect();
        const canvasX = e.clientX - rect.left;
        const canvasY = e.clientY - rect.top;
        const worldCoords = this.canvasToWorldCoordinates(canvasX, canvasY);
        
        // Set goal input fields
        this.elements.goalX.value = worldCoords.x.toFixed(2);
        this.elements.goalY.value = worldCoords.y.toFixed(2);
        this.elements.goalYaw.value = '0'; // Default yaw
        
        // Visual feedback
        this.showNavigationStatus(
            `Goal set from map click: (${worldCoords.x.toFixed(2)}, ${worldCoords.y.toFixed(2)})`,
            'info'
        );
        
        // Auto-hide message
        setTimeout(() => {
            this.hideNavigationStatus();
        }, 2000);
        
        console.log(`🎯 Goal set from map click: (${worldCoords.x.toFixed(2)}, ${worldCoords.y.toFixed(2)})`);
    }
    
    // ==================== END NAVIGATION METHODS ====================
    
    // Debug method to check available navigation interfaces
    checkNavigationInterfaces() {
        console.log('🔍 Checking available navigation interfaces...');
        
        // Check available topics
        this.ros.getTopics((topics) => {
            console.log('📋 Available topics:', topics);
            const navTopics = topics.filter(topic => 
                topic.includes('navigate') || 
                topic.includes('move_base') || 
                topic.includes('goal') ||
                topic.includes('cmd_vel')
            );
            console.log('🎯 Navigation-related topics:', navTopics);
        });
        
        // Check available action servers
        this.ros.getActionServers((servers) => {
            console.log('🎭 Available action servers:', servers);
            const navServers = servers.filter(server => 
                server.includes('navigate') || 
                server.includes('move_base')
            );
            console.log('🎯 Navigation-related action servers:', navServers);
        });
        
        // Check if our specific action server exists
        const testClient = new ROSLIB.ActionClient({
            ros: this.ros,
            serverName: `/${this.selectedAgv}/navigate_to_pose`,
            actionName: 'nav2_msgs/NavigateToPose'
        });
        
        testClient.on('connect', () => {
            console.log(`✅ Action server /${this.selectedAgv}/navigate_to_pose is available`);
        });
        
        testClient.on('error', (error) => {
            console.error(`❌ Action server /${this.selectedAgv}/navigate_to_pose error:`, error);
        });
        
        setTimeout(() => {
            console.log(`🔍 Test action client connected: ${testClient.isConnected}`);
        }, 2000);
    }
}

// Initialize the AGV Control Center when the page loads
document.addEventListener('DOMContentLoaded', () => {
    window.agvControl = new AGVControlCenter();
});

// Prevent default behavior for arrow keys to avoid page scrolling
document.addEventListener('keydown', (e) => {
    if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space'].includes(e.code)) {
        e.preventDefault();
    }
});