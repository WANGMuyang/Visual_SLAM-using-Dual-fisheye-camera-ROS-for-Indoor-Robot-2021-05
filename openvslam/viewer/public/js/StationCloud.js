const STATION_CLOUD_SIZE = 5000;

class StationCloud {
    constructor() {

        this.stationcloudMaterial = new THREE.PointsMaterial({
            station_size: property.StationSize,
            station_vertexColors: true/*,color:"rgb(255,255,0)"*/
        });


        this.station_clouds = [];
        this.station_cloudGeometries = [];
        this.station_cloudGeometryCaches = [];

        this.StationColors = [];

        this.totalStationCnt = 0;  // number of drew point, increase only
        this.nValidStation = 0;  // number of points in SLAM
        this.station_vertexIds = {};  // vertex(Viewer상의 점)의 ID 목록, 3D점의 ID로 참조 가능
        this.station_discardedPool = [];  // points removed from SLAM are not removed in viewer, will be reused when new point added
        this.station_discardedPoolSize = 0; // size of discardedPool

        this.prevReferenceStationIds = [];

        this.POOL_STATION_COORDS = [0, 0, -100000]; // position of pooled point
    }


    // private methods
    addStation(id, x, y, z, r, g, b) {
        // calc point coordinate
        let vector = new THREE.Vector3();
        vector.x = x;
        vector.y = y;
        vector.z = z;

        if (this.station_discardedPoolSize > 0) {
            let vertexId = this.station_discardedPool.pop();
            this.station_vertexIds[id] = vertexId;
            this.station_discardedPoolSize--;

            let cloudIdx = vertexId / STATION_CLOUD_SIZE | 0;
            let StationIdxInCloud = vertexId % STATION_CLOUD_SIZE;

            // reset point coordinate
            if (this.station_cloudGeometries[cloudIdx].vertices[StationIdxInCloud] === undefined) {
                console.error("The point is not in cloud geometry.")
            }
            this.station_cloudGeometries[cloudIdx].vertices[StationIdxInCloud].x = vector.x;
            this.station_cloudGeometries[cloudIdx].vertices[StationIdxInCloud].y = vector.y;
            this.station_cloudGeometries[cloudIdx].vertices[StationIdxInCloud].z = vector.z;
            //this.station_cloudGeometries[cloudIdx].colors[StationIdxInCloud] = new THREE.Color("rgb(" + r + "," + g + "," + b + ")");
            this.station_cloudGeometries[cloudIdx].colors[StationIdxInCloud] = new THREE.Color("rgb(0, 255, 0)");
            this.station_cloudGeometries[cloudIdx].verticesNeedUpdate = true;

        } else {

            this.station_vertexIds[id] = this.totalStationCnt;
            this.totalStationCnt++;

            let cloudIdx = this.totalStationCnt / STATION_CLOUD_SIZE | 0;
            // Create geometry if previous geometry is full.
            if (!this.station_cloudGeometries[cloudIdx]) {
                this.station_cloudGeometries[cloudIdx] = new THREE.Geometry();
            }
            // add point coordinate
            let vertex = new THREE.Vector3(vector.x, vector.y, vector.z);
            this.station_cloudGeometries[cloudIdx].vertices.push(vertex);
            //this.station_cloudGeometries[cloudIdx].colors.push(new THREE.Color("rgb(" + r + "," + g + "," + b + ")"));
            this.station_cloudGeometries[cloudIdx].colors.push(new THREE.Color("rgb(0, 255, 0)"));
            stationUpdateFlag = true;
        }
    }

    changeStationPos(id, x, y, z) {
        let idx = this.station_vertexIds[id];
        let cloudIdx = idx / STATION_CLOUD_SIZE | 0;
        let StationIdx = idx % STATION_CLOUD_SIZE;

        // calc point coordinate
        let vector = new THREE.Vector3();
        vector.x = x;
        vector.y = y;
        vector.z = z;

        if (this.station_cloudGeometries[cloudIdx].vertices[StationIdx] === undefined) {
            this.station_cloudGeometries[cloudIdx].vertices[StationIdx] = new THREE.Vector3();
        }
        this.station_cloudGeometries[cloudIdx].vertices[StationIdx].x = vector.x;
        this.station_cloudGeometries[cloudIdx].vertices[StationIdx].y = vector.y;
        this.station_cloudGeometries[cloudIdx].vertices[StationIdx].z = vector.z;
        this.station_cloudGeometries[cloudIdx].verticesNeedUpdate = true;
    }

    changeStationColor(id, r, g, b) {
        let vertexId = this.station_vertexIds[id];
        if (vertexId === undefined) return;//reference_keyframe에서 해제됨과 동시에 삭제될 수도 있음
        let StationIdx = vertexId / STATION_CLOUD_SIZE | 0;
        let vertexIdxOnCloud = vertexId % STATION_CLOUD_SIZE;
        this.station_cloudGeometries[StationIdx].colors[vertexIdxOnCloud] = new THREE.Color("rgb(" + r + "," + g + "," + b + ")");
        this.station_cloudGeometries[StationIdx].colorsNeedUpdate = true;
    }


    // public methods
    updateStation(id, x, y, z, r, g, b) {

        if (this.station_vertexIds[id] === undefined || this.station_vertexIds[id] < 0) {
            this.addStation(id, x, y, z, r, g, b);
            this.nValidStation++;
        } else {
            this.changeStationPos(id, x, y, z);
            this.changeStationColor(id, r, g, b);
        }
        this.StationColors[id] = [r, g, b];
    }

    removeStation(id) {
        if (!(id in this.station_vertexIds)) {
            return;
        }
        // Move point to pool(origin)
        this.changeStationPos(id, this.POOL_POINT_COORDS[0], this.POOL_POINT_COORDS[1], this.POOL_POINT_COORDS[2]);
        let vertexIdx = this.station_vertexIds[id];
        // Do nothing if point has been already removed.
        if (vertexIdx < 0) {
            return;
        }
        this.station_vertexIds[id] = -1;
        this.station_discardedPool.push(vertexIdx);
        this.station_discardedPoolSize++;
        this.nValidStation--;
    }

    colorizeReferenceStations(referenceStationIds) {

        for (let id of referenceStationIds) {
            stationCloud.changeStationColor(id, REFERENCE_STATION_COLOR[0], REFERENCE_STATION_COLOR[1], REFERENCE_STATION_COLOR[2]);
            this.prevReferenceStationIds.splice(id, 1);
        }
        for (let id of this.prevReferenceStationIds) {
            let color = this.stationCloud[id];
            if (color !== undefined) {
                stationCloud.changeStationColor(id, color[0], color[1], color[2]);
            }
        }
        this.prevReferencePointIds = referencePointIds;
    }

    updateStationInScene(scene) {
        for (let i = 0; i < this.station_cloudGeometries.length; i++) {

            if (this.station_clouds[i]) {
                scene.remove(this.station_clouds[i]);
                this.station_cloudGeometryCaches[i].dispose();
            }

            try {
                this.station_cloudGeometryCaches[i] = this.station_cloudGeometries[i].clone();
                this.station_cloudGeometryCaches[i].colors = this.station_cloudGeometries[i].colors; // should use concat?
                this.station_clouds[i] = new THREE.Points(this.station_cloudGeometryCaches[i], this.stationcloudMaterial);
                scene.add(this.station_clouds[i]);
            } catch (e) {
                console.error("error while cloning CloudGeometry: " + e);
                console.error(i + ", " + this.station_cloudGeometries[i]);
            }
        }
    }

    setStationSize(val) {
        this.stationcloudMaterial.station_size = val
    }
}
