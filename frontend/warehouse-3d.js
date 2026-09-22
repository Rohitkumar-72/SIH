 /* ============================================================
   AM-CORD AI
   REAL 3D WAREHOUSE
   Babylon.js implementation

   Does NOT replace app.js.
   Reads:
       APP_STATE.robots
       APP_STATE.tasks
       APP_STATE.viewMode
       APP_STATE.showTrails
       APP_STATE.showThoughts
       APP_STATE.selectedRobotId
   ============================================================ */

(function () {

    "use strict";

    const Warehouse3D = {

        engine: null,
        scene: null,
        camera: null,

        canvas: null,
        container: null,

        initialized: false,

        robotMeshes: new Map(),
        robotLabels: new Map(),
        robotTrails: new Map(),
        trafficMarkers: new Map(),

        selectedMesh: null,

        highlightLayer: null,
        gizmoManager: null,

        measurement: {
            active: false,
            firstPoint: null,
            secondPoint: null,
            line: null,
            label: null
        },

        warehouse: {
            width: 45,
            depth: 60,
            wallHeight: 5
        },

        materials: {},

        lastRobotUpdate: 0,
        gridMesh: null,
        gridVisible: true,
        trafficVisible: false,
        editMode: false,
        pathKeys: new Map(),

        /* =====================================================
           INIT
           ===================================================== */

        init(containerId) {

            if (this.initialized) {
                return;
            }

            this.container =
                document.getElementById(containerId || "babylonWarehouse3D");

            if (!this.container) {
                console.warn(
                    "[Warehouse3D] Container not found."
                );
                return;
            }

            this.createCanvas();

            this.engine = new BABYLON.Engine(
                this.canvas,
                true,
                {
                    preserveDrawingBuffer: true,
                    stencil: true,
                    antialias: true
                }
            );

            this.scene = new BABYLON.Scene(this.engine);

            this.scene.clearColor =
                new BABYLON.Color4(0.025, 0.035, 0.05, 1);

            this.createCamera();
            this.createLights();
            this.createMaterials();

            this.createWarehouse();
            this.createGrid();
            this.createOrientation();
            this.createTools();

            this.setupPointerEvents();

            this.initialized = true;

            this.engine.runRenderLoop(() => {

                if (!this.scene) {
                    return;
                }

                this.updateRobotMeshes();

                this.scene.render();
            });

            window.addEventListener("resize", () => {
                this.resize();
            });

            this.startStateWatcher();

            console.log(
                "[Warehouse3D] Babylon.js 3D warehouse initialized."
            );
        },

        mount(containerOrId) {
            const target = typeof containerOrId === "string"
                ? document.getElementById(containerOrId)
                : containerOrId;

            if (!target || !this.canvas) return false;

            if (this.canvas.parentElement && this.canvas.parentElement !== target) {
                this.canvas.parentElement.removeChild(this.canvas);
            }
            if (this.infoPanel?.parentElement && this.infoPanel.parentElement !== target) {
                this.infoPanel.parentElement.removeChild(this.infoPanel);
            }

            this.container = target;
            target.appendChild(this.canvas);
            if (this.infoPanel) target.appendChild(this.infoPanel);
            this.resize();
            return true;
        },

        unmount() {
            if (this.canvas?.parentElement) this.canvas.parentElement.removeChild(this.canvas);
            if (this.infoPanel?.parentElement) this.infoPanel.parentElement.removeChild(this.infoPanel);
            this.container = null;
        },

        /* =====================================================
           CANVAS
           ===================================================== */

        createCanvas() {

            this.canvas = document.createElement("canvas");

            this.canvas.id = "babylonWarehouseCanvas";

            this.canvas.style.width = "100%";
            this.canvas.style.height = "100%";
            this.canvas.style.display = "block";

            this.container.appendChild(this.canvas);
        },

        /* =====================================================
           CAMERA
           ===================================================== */

        createCamera() {

            this.camera = new BABYLON.ArcRotateCamera(
                "warehouseCamera",

                -Math.PI / 2,

                Math.PI / 3.2,

                75,

                new BABYLON.Vector3(
                    0,
                    0,
                    0
                ),

                this.scene
            );

            this.camera.attachControl(
                this.canvas,
                true
            );

            /*
             * Zoom
             */
            this.camera.lowerRadiusLimit = 15;
            this.camera.upperRadiusLimit = 130;

            /*
             * Vertical angle
             */
            this.camera.lowerBetaLimit = 0.15;
            this.camera.upperBetaLimit = Math.PI / 2.05;

            /*
             * Smoother camera
             */
            this.camera.wheelDeltaPercentage = 0.01;

            /*
             * Panning
             */
            this.camera.panningSensibility = 80;

            /*
             * Don't allow camera below warehouse
             */
            this.camera.minZ = 0.1;
        },

        resetCamera() {

            if (!this.camera) {
                return;
            }

            this.camera.alpha = -Math.PI / 2;
            this.camera.beta = Math.PI / 3.2;
            this.camera.radius = 75;

            this.camera.target =
                new BABYLON.Vector3(0, 0, 0);
        },

        /* =====================================================
           LIGHTS
           ===================================================== */

        createLights() {

            const hemi =
                new BABYLON.HemisphericLight(
                    "warehouseHemiLight",
                    new BABYLON.Vector3(0, 1, 0),
                    this.scene
                );

            hemi.intensity = 0.75;

            const directional =
                new BABYLON.DirectionalLight(
                    "warehouseSun",
                    new BABYLON.Vector3(
                        -0.5,
                        -1,
                        -0.4
                    ),
                    this.scene
                );

            directional.position =
                new BABYLON.Vector3(
                    20,
                    40,
                    20
                );

            directional.intensity = 1.1;

            /*
             * Shadows
             */
            this.shadowGenerator =
                new BABYLON.ShadowGenerator(
                    2048,
                    directional
                );

            this.shadowGenerator.useBlurExponentialShadowMap = true;
            this.shadowGenerator.blurKernel = 32;
        },

        /* =====================================================
           MATERIALS
           ===================================================== */

        createMaterials() {

            this.materials.floor =
                this.makeMaterial(
                    "floorMaterial",
                    "#111820"
                );

            this.materials.wall =
                this.makeMaterial(
                    "wallMaterial",
                    "#202a35"
                );

            this.materials.rack =
                this.makeMaterial(
                    "rackMaterial",
                    "#344454"
                );

            this.materials.rackTop =
                this.makeMaterial(
                    "rackTopMaterial",
                    "#526579"
                );

            this.materials.loading =
                this.makeMaterial(
                    "loadingMaterial",
                    "#6b5a1f"
                );

            this.materials.charging =
                this.makeMaterial(
                    "chargingMaterial",
                    "#163f35"
                );

            this.materials.obstacle =
                this.makeMaterial(
                    "obstacleMaterial",
                    "#502d32"
                );

            this.materials.robot =
                this.makeMaterial(
                    "robotMaterial",
                    "#2367d1"
                );

            this.materials.robotSelected =
                this.makeMaterial(
                    "robotSelectedMaterial",
                    "#2de2e6"
                );

            this.materials.robotWheel =
                this.makeMaterial(
                    "robotWheelMaterial",
                    "#111111"
                );

            this.materials.path =
                this.makeMaterial(
                    "pathMaterial",
                    "#29d3ff"
                );

            this.materials.grid =
                this.makeMaterial(
                    "gridMaterial",
                    "#27333e"
                );

            this.materials.measure =
                this.makeMaterial(
                    "measureMaterial",
                    "#ffffff"
                );
        },

        makeMaterial(name, hex) {

            const material =
                new BABYLON.StandardMaterial(
                    name,
                    this.scene
                );

            material.diffuseColor =
                BABYLON.Color3.FromHexString(hex);

            material.specularColor =
                new BABYLON.Color3(
                    0.15,
                    0.15,
                    0.15
                );

            return material;
        },

        /* =====================================================
           WAREHOUSE
           ===================================================== */

        createWarehouse() {

            const width = this.warehouse.width;
            const depth = this.warehouse.depth;

            /*
             * Floor
             */
            const floor =
                BABYLON.MeshBuilder.CreateBox(
                    "warehouseFloor",
                    {
                        width: width,
                        height: 0.25,
                        depth: depth
                    },
                    this.scene
                );

            floor.position.y = -0.125;
            floor.material = this.materials.floor;

            floor.receiveShadows = true;

            /*
             * Walls
             */
            this.createWall(
                "wallNorth",
                width,
                this.warehouse.wallHeight,
                0.3,
                0,
                this.warehouse.wallHeight / 2,
                -depth / 2
            );

            this.createWall(
                "wallSouth",
                width,
                this.warehouse.wallHeight,
                0.3,
                0,
                this.warehouse.wallHeight / 2,
                depth / 2
            );

            this.createWall(
                "wallWest",
                0.3,
                this.warehouse.wallHeight,
                depth,
                -width / 2,
                this.warehouse.wallHeight / 2,
                0
            );

            this.createWall(
                "wallEast",
                0.3,
                this.warehouse.wallHeight,
                depth,
                width / 2,
                this.warehouse.wallHeight / 2,
                0
            );

            // The application owns the canonical rack layout.  Fall back to a
            // compact seed layout only when the app has not loaded yet.
            const shelves = window.WAREHOUSE_CONFIG?.shelves || [];
            const rackData = shelves.length > 0 ? shelves : [
                { id: "RACK-W01", name: "Rack W-01", x: -17, y: -20, w: 6, depth_3d: 2.2 },
                { id: "RACK-W02", name: "Rack W-02", x: -8, y: -20, w: 6, depth_3d: 2.2 },
                { id: "RACK-E01", name: "Rack E-01", x: 10, y: 0, w: 6, depth_3d: 2.2 }
            ];
            rackData.forEach(rack => this.createRackFromConfig(rack));


            /*
             * Loading area
             */
            this.createLoadingArea(
                "LoadingArea-A",
                -14,
                22
            );

            this.createLoadingArea(
                "LoadingArea-B",
                14,
                22
            );

            this.createZone("UnloadingArea", "UNLOADING", 0, -25, 10, 5, "#3b2c18");
            this.createZone("SafetyZone", "SAFETY ZONE", 0, 8, 8, 4, "#5b2525");
            this.createEntrance("WarehouseEntrance", 0, -29.8);
            this.createAisleMarkings();

            /*
             * Charging stations
             */
            this.createChargingStation(
                "Charging-01",
                -19,
                18
            );

            this.createChargingStation(
                "Charging-02",
                -15,
                18
            );

            this.createChargingStation(
                "Charging-03",
                15,
                18
            );

            this.createChargingStation(
                "Charging-04",
                19,
                18
            );

            /*
             * Obstacles
             */
            this.createObstacle(
                "Obstacle-01",
                -21,
                -8
            );

            this.createObstacle(
                "Obstacle-02",
                21,
                -8
            );
        },

        createRackFromConfig(rackData) {
            const rack = this.createRack(
                rackData.id || rackData.code || "Rack",
                Number(rackData.x) || 0,
                Number(rackData.y ?? rackData.z) || 0,
                Number(rackData.w) || 3.92,
                Number(rackData.depth_3d) || 2.2
            );
            rack.metadata = {
                type: "rack",
                id: rackData.id || rackData.code,
                name: rackData.name || rackData.id,
                zone: rackData.zone || "WAREHOUSE",
                position: { x: Number(rackData.x) || 0, y: 0, z: Number(rackData.y ?? rackData.z) || 0 }
            };
            rack.getChildMeshes().forEach(mesh => {
                mesh.metadata = { ...rack.metadata };
            });
            this.createWorldLabel(rackData.name || rackData.code || rackData.id, Number(rackData.x) || 0, 5.6, Number(rackData.y ?? rackData.z) || 0);
            return rack;
        },

        createZone(name, label, x, z, width, depth, color) {
            const zone = BABYLON.MeshBuilder.CreateBox(name, { width, height: 0.04, depth }, this.scene);
            zone.position = new BABYLON.Vector3(x, 0.04, z);
            zone.material = this.makeMaterial(`${name}Material`, color);
            zone.metadata = { type: "zone", id: name, name: label, position: { x, y: 0, z } };
            this.createWorldLabel(label, x, 0.25, z);
            return zone;
        },

        createEntrance(name, x, z) {
            const entrance = BABYLON.MeshBuilder.CreateBox(name, { width: 8, height: 0.05, depth: 0.5 }, this.scene);
            entrance.position = new BABYLON.Vector3(x, 0.06, z);
            entrance.material = this.materials.loading;
            entrance.metadata = { type: "entrance", id: name, name: "ENTRANCE / EXIT", position: { x, y: 0, z } };
            this.createWorldLabel("ENTRANCE / EXIT", x, 0.25, z + 1.2);
        },

        createAisleMarkings() {
            const markings = [];
            for (const corridor of (window.WAREHOUSE_CONFIG?.corridors || [])) {
                const isVertical = corridor.orientation === "V";
                markings.push({
                    x: isVertical ? corridor.x : 0,
                    z: isVertical ? 0 : corridor.y,
                    width: isVertical ? 0.08 : this.warehouse.width - 2,
                    depth: isVertical ? this.warehouse.depth - 2 : 0.08,
                    label: corridor.name
                });
            }
            markings.forEach((marking, index) => {
                const line = BABYLON.MeshBuilder.CreateBox(`aisleMarking-${index}`, {
                    width: marking.width,
                    height: 0.025,
                    depth: marking.depth
                }, this.scene);
                line.position = new BABYLON.Vector3(marking.x, 0.08, marking.z);
                line.material = this.materials.measure;
                line.alpha = 0.65;
                line.metadata = { type: "aisle", id: marking.label, name: marking.label, position: { x: marking.x, y: 0, z: marking.z } };
                this.createWorldLabel(marking.label, marking.x, 0.2, marking.z);
            });
        },

        createWall(
            name,
            width,
            height,
            depth,
            x,
            y,
            z
        ) {

            const wall =
                BABYLON.MeshBuilder.CreateBox(
                    name,
                    {
                        width,
                        height,
                        depth
                    },
                    this.scene
                );

            wall.position =
                new BABYLON.Vector3(
                    x,
                    y,
                    z
                );

            wall.material =
                this.materials.wall;

            wall.receiveShadows = true;

            return wall;
        },

        /* =====================================================
           RACKS
           ===================================================== */

        createRack(name, x, z, rackWidth = 6, rackDepth = 2) {

            const rackRoot =
                new BABYLON.TransformNode(
                    name,
                    this.scene
                );

            rackRoot.position.x = x;
            rackRoot.position.z = z;

            const width = rackWidth;
            const depth = rackDepth;
            const height = 5;

            /*
             * Vertical posts
             */
            const postPositions = [
                [-width / 2, -depth / 2],
                [width / 2, -depth / 2],
                [-width / 2, depth / 2],
                [width / 2, depth / 2]
            ];

            postPositions.forEach(
                ([px, pz], index) => {

                    const post =
                        BABYLON.MeshBuilder.CreateBox(
                            `${name}-post-${index}`,
                            {
                                width: 0.25,
                                height,
                                depth: 0.25
                            },
                            this.scene
                        );

                    post.parent = rackRoot;

                    post.position =
                        new BABYLON.Vector3(
                            px,
                            height / 2,
                            pz
                        );

                    post.material =
                        this.materials.rack;
                }
            );

            /*
             * Shelves
             */
            const shelfLevels = [
                0.8,
                2.0,
                3.2,
                4.4
            ];

            shelfLevels.forEach(
                (level, index) => {

                    const shelf =
                        BABYLON.MeshBuilder.CreateBox(
                            `${name}-shelf-${index}`,
                            {
                                width: width,
                                height: 0.18,
                                depth: depth
                            },
                            this.scene
                        );

                    shelf.parent = rackRoot;

                    shelf.position =
                        new BABYLON.Vector3(
                            0,
                            level,
                            0
                        );

                    shelf.material =
                        this.materials.rackTop;

                    this.shadowGenerator
                        ?.addShadowCaster(shelf);
                }
            );

            /*
             * Rack boxes / inventory
             */
            shelfLevels.forEach(
                (level, levelIndex) => {

                    for (
                        let i = 0;
                        i < 3;
                        i++
                    ) {

                        const box =
                            BABYLON.MeshBuilder.CreateBox(
                                `${name}-box-${levelIndex}-${i}`,
                                {
                                    width: 1.3,
                                    height: 0.7,
                                    depth: 1.3
                                },
                                this.scene
                            );

                        box.parent = rackRoot;

                        box.position =
                            new BABYLON.Vector3(
                                -1.7 + i * 1.7,
                                level + 0.45,
                                0
                            );

                        box.material =
                            this.materials.rackTop;
                    }
                }
            );

            return rackRoot;
        },

        /* =====================================================
           LOADING AREA
           ===================================================== */

        createLoadingArea(name, x, z) {

            const area =
                BABYLON.MeshBuilder.CreateBox(
                    name,
                    {
                        width: 8,
                        height: 0.05,
                        depth: 5
                    },
                    this.scene
                );

            area.position =
                new BABYLON.Vector3(
                    x,
                    0.03,
                    z
                );

            area.material =
                this.materials.loading;

            /*
             * Loading columns
             */
            for (let i = -1; i <= 1; i++) {

                const column =
                    BABYLON.MeshBuilder.CreateBox(
                        `${name}-column-${i}`,
                        {
                            width: 0.3,
                            height: 3,
                            depth: 0.3
                        },
                        this.scene
                    );

                column.position =
                    new BABYLON.Vector3(
                        x + i * 3,
                        1.5,
                        z - 2
                    );

                column.material =
                    this.materials.wall;
            }
        },

        /* =====================================================
           CHARGING
           ===================================================== */

        createChargingStation(name, x, z) {

            const station =
                BABYLON.MeshBuilder.CreateBox(
                    name,
                    {
                        width: 3,
                        height: 0.08,
                        depth: 3
                    },
                    this.scene
                );

            station.position =
                new BABYLON.Vector3(
                    x,
                    0.05,
                    z
                );

            station.material =
                this.materials.charging;
            station.metadata = {
                type: "charging station",
                id: name,
                name,
                position: { x, y: 0, z }
            };
            this.createWorldLabel(name.replace("-", " ").toUpperCase(), x, 0.25, z);

            /*
             * Charging post
             */
            const post =
                BABYLON.MeshBuilder.CreateBox(
                    `${name}-post`,
                    {
                        width: 0.5,
                        height: 1.8,
                        depth: 0.3
                    },
                    this.scene
                );

            post.position =
                new BABYLON.Vector3(
                    x,
                    0.9,
                    z - 1.2
                );

            post.material =
                this.materials.charging;
            post.metadata = station.metadata;
        },

        /* =====================================================
           OBSTACLES
           ===================================================== */

        createObstacle(name, x, z) {

            const obstacle =
                BABYLON.MeshBuilder.CreateBox(
                    name,
                    {
                        width: 2.5,
                        height: 1.2,
                        depth: 2.5
                    },
                    this.scene
                );

            obstacle.position =
                new BABYLON.Vector3(
                    x,
                    0.6,
                    z
                );

            obstacle.material =
                this.materials.obstacle;
        },

        /* =====================================================
           GRID
           ===================================================== */

        createGrid() {

            const lines = [];

            const width = this.warehouse.width;
            const depth = this.warehouse.depth;

            const step = 1;

            /*
             * X direction
             */
            for (
                let x = -width / 2;
                x <= width / 2;
                x += step
            ) {

                lines.push([
                    new BABYLON.Vector3(
                        x,
                        0.02,
                        -depth / 2
                    ),

                    new BABYLON.Vector3(
                        x,
                        0.02,
                        depth / 2
                    )
                ]);
            }

            /*
             * Z direction
             */
            for (
                let z = -depth / 2;
                z <= depth / 2;
                z += step
            ) {

                lines.push([
                    new BABYLON.Vector3(
                        -width / 2,
                        0.02,
                        z
                    ),

                    new BABYLON.Vector3(
                        width / 2,
                        0.02,
                        z
                    )
                ]);
            }

            const grid =
                BABYLON.MeshBuilder.CreateLineSystem(
                    "warehouseGrid",
                    {
                        lines: lines
                    },
                    this.scene
                );

            grid.color =
                BABYLON.Color3.FromHexString(
                    "#26323d"
                );
            this.gridMesh = grid;
        },

        /* =====================================================
           AXES
           ===================================================== */

        createOrientation() {

            /*
             * X axis
             */
            const xAxis =
                BABYLON.MeshBuilder.CreateLines(
                    "XAxis",
                    {
                        points: [
                            new BABYLON.Vector3(0, 0.1, 0),
                            new BABYLON.Vector3(5, 0.1, 0)
                        ]
                    },
                    this.scene
                );

            xAxis.color =
                new BABYLON.Color3(1, 0.15, 0.15);

            /*
             * Z axis
             */
            const zAxis =
                BABYLON.MeshBuilder.CreateLines(
                    "ZAxis",
                    {
                        points: [
                            new BABYLON.Vector3(0, 0.1, 0),
                            new BABYLON.Vector3(0, 0.1, 5)
                        ]
                    },
                    this.scene
                );

            zAxis.color =
                new BABYLON.Color3(
                    0.15,
                    0.6,
                    1
                );

            /*
             * Y axis
             */
            const yAxis =
                BABYLON.MeshBuilder.CreateLines(
                    "YAxis",
                    {
                        points: [
                            new BABYLON.Vector3(0, 0, 0),
                            new BABYLON.Vector3(0, 5, 0)
                        ]
                    },
                    this.scene
                );

            yAxis.color =
                new BABYLON.Color3(
                    0.2,
                    1,
                    0.2
                );
        },

        /* =====================================================
           TOOLS
           ===================================================== */

        createTools() {

            /*
             * Highlight selected object
             */
            this.highlightLayer =
                new BABYLON.HighlightLayer(
                    "warehouseHighlight",
                    this.scene
                );

            /*
             * Gizmo manager
             */
            this.gizmoManager =
                new BABYLON.GizmoManager(
                    this.scene
                );

            this.gizmoManager.positionGizmoEnabled = true;
            this.gizmoManager.rotationGizmoEnabled = true;

            /*
             * Initially disabled
             */
            this.gizmoManager.attachableMeshes = [];

            this.createInteractionUi();

            /*
             * We don't show gizmos for normal
             * warehouse objects unless selected.
             */
        },

        createInteractionUi() {
            const info = document.createElement("div");
            info.className = "babylon-info-panel";
            info.innerHTML = '<div class="babylon-info-title">Nothing selected</div><div class="babylon-info-row"><span>Click a warehouse object</span></div>';
            this.container.appendChild(info);
            this.infoPanel = info;
        },

        toggleGrid() {
            this.gridVisible = !this.gridVisible;
            if (this.gridMesh) this.gridMesh.isVisible = this.gridVisible;
            return this.gridVisible;
        },

        toggleMeasurement() {
            if (this.measurement.active) {
                this.disableMeasurement();
                return false;
            }
            this.enableMeasurement();
            return true;
        },

        setTrafficVisible(visible) {
            this.trafficVisible = Boolean(visible);
            this.trafficMarkers.forEach(marker => {
                marker.isVisible = this.trafficVisible;
            });
        },

        updateTrafficMarkers() {
            const zones = window.AM_CORD_TRAFFIC?.zones || [];
            const activeIds = new Set(zones.map(zone => zone.robotId));
            zones.forEach(zone => {
                let marker = this.trafficMarkers.get(zone.robotId);
                if (!marker) {
                    marker = BABYLON.MeshBuilder.CreateTorus(`traffic-${zone.robotId}`, {
                        diameter: 2.2,
                        thickness: 0.08,
                        tessellation: 32
                    }, this.scene);
                    marker.rotation.x = Math.PI / 2;
                    marker.material = this.makeMaterial(`traffic-${zone.robotId}-material`, "#22c55e");
                    this.trafficMarkers.set(zone.robotId, marker);
                }
                const color = zone.density >= 0.75 ? "#ef4444" : zone.density >= 0.35 ? "#f59e0b" : "#22c55e";
                marker.position = new BABYLON.Vector3(zone.x, 0.12, zone.y);
                marker.material.diffuseColor = BABYLON.Color3.FromHexString(color);
                marker.material.emissiveColor = BABYLON.Color3.FromHexString(color);
                marker.isVisible = this.trafficVisible && Boolean(window.AM_CORD_TRAFFIC?.enabled);
            });
            this.trafficMarkers.forEach((marker, id) => {
                if (!activeIds.has(id)) marker.isVisible = false;
            });
        },

        setCameraView(view) {
            const views = {
                top: { alpha: -Math.PI / 2, beta: 0.08, radius: 70 },
                front: { alpha: -Math.PI / 2, beta: Math.PI / 2.15, radius: 65 },
                side: { alpha: 0, beta: Math.PI / 2.15, radius: 65 },
                perspective: { alpha: -Math.PI / 2, beta: Math.PI / 3.2, radius: 75 }
            };
            const target = views[view] || views.perspective;
            this.camera.alpha = target.alpha;
            this.camera.beta = target.beta;
            this.camera.radius = target.radius;
            this.camera.target = new BABYLON.Vector3(0, 0, 0);
        },

        updateGizmoAttachment() {
            if (!this.gizmoManager) return;
            const mesh = this.selectedMesh;
            const isRobot = mesh?.metadata?.robotId;
            if (this.editMode && mesh && !isRobot) {
                this.gizmoManager.attachToMesh(mesh);
            } else {
                this.gizmoManager.attachToMesh(null);
            }
        },

        /* =====================================================
           POINTER / SELECTION
           ===================================================== */

        setupPointerEvents() {

            this.scene.onPointerObservable.add(
                (pointerInfo) => {

                    if (
                        pointerInfo.type !==
                        BABYLON.PointerEventTypes.POINTERPICK
                    ) {
                        return;
                    }

                    const pickInfo =
                        pointerInfo.pickInfo;

                    if (
                        !pickInfo ||
                        !pickInfo.hit ||
                        !pickInfo.pickedMesh
                    ) {
                        return;
                    }

                    const mesh =
                        pickInfo.pickedMesh;

                    if (this.measurement.active) {
                        this.addMeasurementPoint(pickInfo.pickedPoint);
                        return;
                    }

                    /*
                     * Robot selection
                     */
                    if (
                        mesh.metadata &&
                        mesh.metadata.robotId
                    ) {

                        this.selectRobot(
                            mesh.metadata.robotId
                        );

                        return;
                    }

                    /*
                     * Generic object selection
                     */
                    this.selectObject(mesh);
                }
            );
        },

        selectObject(mesh) {

            if (!mesh) {
                return;
            }

            if (this.selectedMesh) {

                this.highlightLayer.removeMesh(
                    this.selectedMesh
                );
            }

            this.selectedMesh = mesh;

            this.highlightLayer.addMesh(
                mesh,
                BABYLON.Color3.FromHexString(
                    "#00e5ff"
                )
            );

            /*
             * Gizmo
             */
            if (
                this.gizmoManager &&
                mesh
            ) {

                this.gizmoManager.attachToMesh(
                        null
                    );
            }

                this.updateSelectionInfo(mesh.metadata || {}, mesh);
                this.updateGizmoAttachment();
        },

        selectRobot(robotId) {

            const robot =
                this.robotMeshes.get(robotId);

            if (!robot) {
                return;
            }

            if (this.selectedMesh) {

                this.highlightLayer.removeMesh(
                    this.selectedMesh
                );
            }

            this.selectedMesh = robot.body;

            this.highlightLayer.addMesh(
                robot.body,
                BABYLON.Color3.FromHexString(
                    "#00ffff"
                )
            );

            /*
             * Inform existing app
             */
            try {

                if (
                    window.APP_STATE &&
                    APP_STATE.robots
                ) {

                    APP_STATE.selectedRobotId =
                        robotId;
                }

            } catch (error) {
                console.warn(error);
            }

            /*
             * Gizmo disabled for AMRs by default
             * because simulation controls their position.
             */
            if (this.gizmoManager) {
                this.updateGizmoAttachment();
            }

            if (window.selectAmr) window.selectAmr(robotId);
            this.updateSelectionInfo({ type: "AMR", id: robotId }, robot.body, robotId);
        },

        updateSelectionInfo(metadata, mesh, robotId = null) {
            if (!this.infoPanel) return;
            const position = mesh?.getAbsolutePosition?.() || mesh?.position || BABYLON.Vector3.Zero();
            if (robotId) {
                const data = (window.APP_STATE?.robots || []).find(robot => robot.id === robotId);
                if (data) {
                    this.infoPanel.innerHTML = `<div class="babylon-info-title">${data.name.toUpperCase()}</div><div class="babylon-info-row"><span>Type</span><strong>AMR</strong></div><div class="babylon-info-row"><span>State</span><strong>${data.state || "IDLE"}</strong></div><div class="babylon-info-row"><span>Task</span><strong>${data.taskId || "None"}</strong></div><div class="babylon-info-row"><span>Speed</span><strong>${Number(data.speed || 0).toFixed(2)} m/s</strong></div><div class="babylon-info-row"><span>Battery</span><strong>${Number(data.battery || 0).toFixed(0)}%</strong></div><div class="babylon-info-row"><span>Position</span><strong>${position.x.toFixed(2)}, ${position.y.toFixed(2)}, ${position.z.toFixed(2)}</strong></div>`;
                    return;
                }
            }
            this.infoPanel.innerHTML = `<div class="babylon-info-title">${metadata.name || metadata.id || "Selected object"}</div><div class="babylon-info-row"><span>Type</span><strong>${metadata.type || "Object"}</strong></div><div class="babylon-info-row"><span>Position</span><strong>${position.x.toFixed(2)}, ${position.y.toFixed(2)}, ${position.z.toFixed(2)}</strong></div>`;
        },

        /* =====================================================
           ROBOT CREATION
           ===================================================== */

        createRobot(robotData) {

            const root =
                new BABYLON.TransformNode(
                    `AMR-${robotData.id}`,
                    this.scene
                );

            /*
             * Body
             */
            const body =
                BABYLON.MeshBuilder.CreateBox(
                    `${robotData.id}-body`,
                    {
                        width: 1.5,
                        height: 0.45,
                        depth: 1.9
                    },
                    this.scene
                );

            body.parent = root;

            body.position.y = 0.4;

            body.material =
                this.makeMaterial(`${robotData.id}-body-material`, robotData.color || "#2367d1");

            /*
             * Top plate
             */
            const top =
                BABYLON.MeshBuilder.CreateBox(
                    `${robotData.id}-top`,
                    {
                        width: 1.1,
                        height: 0.12,
                        depth: 1.3
                    },
                    this.scene
                );

            top.parent = root;

            top.position.y = 0.7;

            top.material =
                this.materials.robotSelected;

            /*
             * Four wheels
             */
            const wheelPositions = [
                [-0.65, -0.65],
                [0.65, -0.65],
                [-0.65, 0.65],
                [0.65, 0.65]
            ];

            wheelPositions.forEach(
                ([x, z], index) => {

                    const wheel =
                        BABYLON.MeshBuilder.CreateCylinder(
                            `${robotData.id}-wheel-${index}`,
                            {
                                diameter: 0.45,
                                height: 0.18,
                                tessellation: 20
                            },
                            this.scene
                        );

                    wheel.parent = root;

                    wheel.rotation.z =
                        Math.PI / 2;

                    wheel.position =
                        new BABYLON.Vector3(
                            x,
                            0.22,
                            z
                        );

                    wheel.material =
                        this.materials.robotWheel;
                }
            );

            /*
             * Front sensor
             */
            const sensor =
                BABYLON.MeshBuilder.CreateCylinder(
                    `${robotData.id}-sensor`,
                    {
                        diameter: 0.25,
                        height: 0.1,
                        tessellation: 20
                    },
                    this.scene
                );

            sensor.parent = root;

            sensor.position =
                new BABYLON.Vector3(
                    0,
                    0.85,
                    -0.5
                );

            sensor.material =
                this.materials.robotSelected;
            const status = BABYLON.MeshBuilder.CreateSphere(
                `${robotData.id}-status`,
                { diameter: 0.18, segments: 12 },
                this.scene
            );
            status.parent = root;
            status.position = new BABYLON.Vector3(0, 0.9, 0.35);
            status.material = this.makeMaterial(`${robotData.id}-status-material`, "#10b981");

            /*
             * Metadata
             */
            body.metadata = {
                robotId: robotData.id
            };

            top.metadata = {
                robotId: robotData.id
            };

            sensor.metadata = {
                robotId: robotData.id
            };

            /*
             * Root metadata
             */
            root.metadata = {
                robotId: robotData.id
            };
            status.metadata = { robotId: robotData.id };

            /*
             * Add shadows
             */
            this.shadowGenerator
                ?.addShadowCaster(body);

            this.shadowGenerator
                ?.addShadowCaster(top);

            /*
             * Label
             */
            const label =
                this.createRobotLabel(
                    robotData.id
                );

            label.parent = root;

            label.position.y = 2.0;

            this.robotLabels.set(
                robotData.id,
                label
            );

            /*
             * Store
             */
            const robotObject = {
                root,
                body,
                top,
                status,
                targetX: 0,
                targetZ: 0,
                currentX: 0,
                currentZ: 0,
                targetRotation: 0
            };

            this.robotMeshes.set(
                robotData.id,
                robotObject
            );

            return robotObject;
        },

        /* =====================================================
           ROBOT LABEL
           ===================================================== */

        createWorldLabel(text, x, y, z) {
            const label = this.createRobotLabel(text);
            label.position = new BABYLON.Vector3(x, y, z);
            label.scaling = new BABYLON.Vector3(0.7, 0.7, 0.7);
            return label;
        },

        createRobotLabel(text) {

            const texture =
                new BABYLON.DynamicTexture(
                    `label-${text}`,
                    {
                        width: 512,
                        height: 128
                    },
                    this.scene,
                    true
                );

            texture.hasAlpha = true;

            const ctx =
                texture.getContext();

            ctx.clearRect(
                0,
                0,
                512,
                128
            );

            ctx.fillStyle =
                "rgba(0,0,0,0.78)";

            ctx.fillRect(
                5,
                5,
                502,
                118
            );

            ctx.fillStyle = "#ffffff";
            ctx.font = "bold 42px Arial";
            ctx.textAlign = "center";
            ctx.textBaseline = "middle";

            ctx.fillText(
                String(text).toUpperCase(),
                256,
                64
            );

            texture.update();

            const material =
                new BABYLON.StandardMaterial(
                    `labelMaterial-${text}`,
                    this.scene
                );

            material.diffuseTexture =
                texture;

            material.opacityTexture =
                texture;

            material.emissiveColor =
                BABYLON.Color3.White();

            material.backFaceCulling = false;

            const plane =
                BABYLON.MeshBuilder.CreatePlane(
                    `labelPlane-${text}`,
                    {
                        width: 3,
                        height: 0.75
                    },
                    this.scene
                );

            plane.billboardMode =
                BABYLON.Mesh.BILLBOARDMODE_ALL;

            plane.material = material;

            return plane;
        },

        /* =====================================================
           ROBOT POSITION UPDATE
           ===================================================== */

        updateRobots(robots) {

            if (!robots) {
                return;
            }

            window.calculateTrafficDensity?.();
            this.updateTrafficMarkers();

            /*
             * Handle both arrays and objects
             */
            let robotArray = [];

            if (Array.isArray(robots)) {

                robotArray = robots;

            } else if (
                typeof robots === "object"
            ) {

                robotArray =
                    Object.values(robots);
            }

            robotArray.forEach(
                robotData => {

                    if (!robotData) {
                        return;
                    }

                    const id =
                        robotData.id ||
                        robotData.robotId;

                    if (!id) {
                        return;
                    }

                    if (
                        !this.robotMeshes.has(id)
                    ) {

                        this.createRobot(
                            robotData
                        );
                    }

                    const robot =
                        this.robotMeshes.get(id);

                    if (robot.status?.material) {
                        const statusColor = robotData.isCharging ? "#22c55e" : robotData.state === "IDLE" ? "#f59e0b" : "#06b6d4";
                        robot.status.material.emissiveColor = BABYLON.Color3.FromHexString(statusColor);
                        robot.status.material.diffuseColor = BABYLON.Color3.FromHexString(statusColor);
                    }

                    /*
                     * Existing app uses x/y
                     *
                     * Babylon:
                     * x -> x
                     * y -> z
                     */
                    const x =
                        Number(robotData.x) || 0;

                    const z =
                        Number(robotData.y) || 0;

                    /*
                     * Convert warehouse coordinates
                     * to Babylon center coordinates.
                     *
                     * If your existing simulation already
                     * uses centered coordinates, remove
                     * these offsets.
                     */
                    robot.targetX = x;
                    robot.targetZ = z;
                   

                    /*
                     * Initial position
                     */
                    if (
                        !robot.initialized
                    ) {

                        robot.currentX =
                            robot.targetX;

                        robot.currentZ =
                            robot.targetZ;

                        robot.initialized =
                            true;
                    }

                    /*
                     * Rotation
                     */
                    if (
                        typeof robotData.theta ===
                        "number"
                    ) {

                        robot.targetRotation =
                            -robotData.theta;
                    }

                    /*
                     * Update label
                     */
                    this.updateRobotLabel(
                        id,
                        robotData
                    );

                    /*
                     * Update path
                     */
                    this.updateRobotPath(
                        robotData
                    );
                }
            );
        },

        /* =====================================================
           ROBOT SMOOTH ANIMATION
           ===================================================== */

        updateRobotMeshes() {

            this.robotMeshes.forEach(
                (robot, id) => {

                    if (!robot.initialized) {
                        return;
                    }

                    const lerp =
                        0.12;

                    robot.currentX +=
                        (
                            robot.targetX -
                            robot.currentX
                        ) * lerp;

                    robot.currentZ +=
                        (
                            robot.targetZ -
                            robot.currentZ
                        ) * lerp;

                    robot.root.position.x =
                        robot.currentX;

                    robot.root.position.z =
                        robot.currentZ;

                    /*
                     * Rotation
                     */
                    const currentRotation =
                        robot.root.rotation.y;

                    let difference =
                        robot.targetRotation -
                        currentRotation;

                    /*
                     * Normalize angle
                     */
                    while (
                        difference > Math.PI
                    ) {
                        difference -=
                            Math.PI * 2;
                    }

                    while (
                        difference < -Math.PI
                    ) {
                        difference +=
                            Math.PI * 2;
                    }

                    robot.root.rotation.y +=
                        difference * 0.12;
                }
            );
        },

        /* =====================================================
           ROBOT LABEL UPDATE
           ===================================================== */

        updateRobotLabel(
            id,
            robotData
        ) {

            const label =
                this.robotLabels.get(id);

            if (!label) {
                return;
            }

            /*
             * Hide thoughts if app says so
             */
            try {

                if (
                    window.APP_STATE &&
                    APP_STATE.showThoughts === false
                ) {

                    label.isVisible = false;

                } else {

                    label.isVisible = true;
                }

            } catch (error) {}

            /*
             * Update label texture
             */
            const texture =
                label.material?.diffuseTexture;

            if (!texture) {
                return;
            }

            const ctx =
                texture.getContext();

            ctx.clearRect(
                0,
                0,
                512,
                128
            );

            ctx.fillStyle =
                "rgba(0,0,0,0.82)";

            ctx.fillRect(
                5,
                5,
                502,
                118
            );

            ctx.fillStyle =
                "#ffffff";

            ctx.font =
                "bold 30px Arial";

            ctx.textAlign =
                "center";

            ctx.textBaseline =
                "middle";

            ctx.fillText(
                String(id).toUpperCase(),
                256,
                30
            );

            ctx.font =
                "20px Arial";

            const battery =
                robotData.battery ??
                "--";

            ctx.fillText(`${String(robotData.state || "IDLE").replaceAll("_", " ")}`, 256, 58);
            ctx.fillText(`Battery: ${battery}%`, 256, 82);
            ctx.fillText(`Speed: ${Number(robotData.speed || 0).toFixed(2)} m/s`, 256, 106);

            texture.update();
        },

        /* =====================================================
           PATHS / TRAILS
           ===================================================== */

        updateRobotPath(robotData) {

            if (!robotData) {
                return;
            }

            let path =
                robotData.path;

            if (!Array.isArray(path)) {
                return;
            }

            try {

                if (
                    window.APP_STATE &&
                    APP_STATE.showTrails === false
                ) {

                    const oldTrail =
                        this.robotTrails.get(
                            robotData.id
                        );

                    if (oldTrail) {
                        oldTrail.isVisible =
                            false;
                    }

                    return;
                }

            } catch (error) {}

            const points = [];
            const breadcrumbPath = Array.isArray(robotData.breadcrumbs) ? robotData.breadcrumbs : [];
            const sourcePath = path.length > 1 ? path : breadcrumbPath;
            const pathKey = JSON.stringify(sourcePath);
            const id = robotData.id;
            if (this.pathKeys.get(id) === pathKey) return;
            this.pathKeys.set(id, pathKey);

            sourcePath.forEach(
                point => {

                    let x;
                    let z;

                    if (
                        Array.isArray(point)
                    ) {

                        x =
                            Number(point[0]) || 0;

                        z =
                            Number(point[1]) || 0;

                    } else if (
                        point &&
                        typeof point === "object"
                    ) {

                        x =
                            Number(
                                point.x
                            ) || 0;

                        z =
                            Number(
                                point.y ??
                                point.z
                            ) || 0;

                    } else {

                        return;
                    }

                    points.push(
                        new BABYLON.Vector3(
                            x,

                            0.12,

                            z 
                               
                        )
                    );
                }
            );

            if (points.length < 2) {
                return;
            }

            const oldTrail =
                this.robotTrails.get(id);

            if (oldTrail) {
                oldTrail.dispose();
            }

            const trail =
                BABYLON.MeshBuilder.CreateLines(
                    `path-${id}`,
                    {
                        points: points
                    },
                    this.scene
                );

            trail.color =
                BABYLON.Color3.FromHexString(
                    "#29d3ff"
                );

            trail.alpha = 0.65;

            this.robotTrails.set(
                id,
                trail
            );
        },

        /* =====================================================
           MEASUREMENT TOOL
           ===================================================== */

        enableMeasurement() {

            this.measurement.active = true;

            this.measurement.firstPoint = null;
            this.measurement.secondPoint = null;

            if (
                this.measurement.line
            ) {

                this.measurement.line.dispose();

                this.measurement.line =
                    null;
            }

            console.log(
                "[Warehouse3D] Measurement mode ON. Click two points."
            );
        },

        disableMeasurement() {

            this.measurement.active = false;
        },

        addMeasurementPoint(point) {

            if (
                !this.measurement.firstPoint
            ) {

                this.measurement.firstPoint =
                    point.clone();

                console.log(
                    "[Warehouse3D] First measurement point selected."
                );

                return;
            }

            this.measurement.secondPoint =
                point.clone();

            this.drawMeasurement();

            this.measurement.active =
                false;
        },

        drawMeasurement() {

            const p1 =
                this.measurement.firstPoint;

            const p2 =
                this.measurement.secondPoint;

            if (!p1 || !p2) {
                return;
            }

            if (
                this.measurement.line
            ) {

                this.measurement.line.dispose();
            }

            this.measurement.line =
                BABYLON.MeshBuilder.CreateLines(
                    "measurementLine",
                    {
                        points: [
                            p1,
                            p2
                        ]
                    },
                    this.scene
                );

            this.measurement.line.color =
                BABYLON.Color3.White();

            const distance =
                BABYLON.Vector3.Distance(
                    p1,
                    p2
                );

            if (this.infoPanel) {
                this.infoPanel.innerHTML = `<div class="babylon-info-title">DISTANCE</div><div class="babylon-info-row"><span>3D distance</span><strong>${distance.toFixed(2)} m</strong></div><div class="babylon-info-row"><span>Point A</span><strong>${p1.x.toFixed(2)}, ${p1.y.toFixed(2)}, ${p1.z.toFixed(2)}</strong></div><div class="babylon-info-row"><span>Point B</span><strong>${p2.x.toFixed(2)}, ${p2.y.toFixed(2)}, ${p2.z.toFixed(2)}</strong></div>`;
            }

            console.log(
                `[Warehouse3D] Distance: ${distance.toFixed(2)} m`
            );
        },

        /* =====================================================
           VISIBILITY
           ===================================================== */

        setVisible(visible) {

            if (!this.container) {
                return;
            }

            this.container.style.display =
                visible ? "block" : "none";

            if (
                this.canvas
            ) {

                this.canvas.style.display =
                    visible ? "block" : "none";
            }

            if (visible) {
                this.resize();
            }
        },

        /* =====================================================
           STATE WATCHER
           ===================================================== */

        startStateWatcher() {

            setInterval(
                () => {

                    try {

                        if (
                            !window.APP_STATE
                        ) {
                            return;
                        }

                        /*
                         * 2D / 3D
                         */
                        const is3D =
                            String(
                                APP_STATE.viewMode
                            ).toUpperCase() ===
                            "3D";

                        this.setVisible(
                            is3D
                        );

                        /*
                         * Robot state
                         */
                        if (
                            APP_STATE.robots
                        ) {

                            this.updateRobots(
                                APP_STATE.robots
                            );
                        }

                    } catch (error) {

                        /*
                         * Do not break main dashboard
                         */
                    }

                },
                100
            );
        },

        /* =====================================================
           RESIZE
           ===================================================== */

        resize() {

            if (this.engine) {
                this.engine.resize();
            }
        },

        /* =====================================================
           DESTROY
           ===================================================== */

        dispose() {

            if (this.engine) {
                this.engine.stopRenderLoop();
                this.engine.dispose();
            }

            this.engine = null;
            this.scene = null;

            this.robotMeshes.clear();
            this.robotLabels.clear();
            this.robotTrails.clear();

            this.initialized = false;
        }
    };


    /* ========================================================
       GLOBAL API
       ======================================================== */

    window.Warehouse3D = Warehouse3D;
    })();


    