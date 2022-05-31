console.log("loaded quadtree.js");

var currentStep = 1
var totalSteps = null
var globalQuadtreeStepsData = null
var maxBBox = null

async function getBBoxes(data) {
    return await Promise.all(data.map(async (o) => {
        return new Promise((resolve, reject) => {
            resolve({
                tl: o.quadtree.top_left,
                br: [o.quadtree.top_left[0] + o.quadtree['length'], o.quadtree.top_left[1] - o.quadtree['length']]
            })
        })
    }));

}

function getMaxBBox(bBoxes) {
    let maxBBox = {
        br: [-Infinity, Infinity],
        tl: [Infinity, -Infinity]
    }

    bBoxes.forEach(function (bBox) {
        if (bBox.tl[0] < maxBBox.tl[0]) maxBBox.tl[0] = bBox.tl[0];
        if (bBox.tl[1] > maxBBox.tl[1]) maxBBox.tl[1] = bBox.tl[1];
        if (bBox.br[0] > maxBBox.br[0]) maxBBox.br[0] = bBox.br[0];
        if (bBox.br[1] < maxBBox.br[1]) maxBBox.br[1] = bBox.br[1];
    })

    return maxBBox
}

function normalizeQuadtree(idx, maxBBox) {
    let data = JSON.parse(JSON.stringify(globalQuadtreeStepsData))

    function navigateQuadtree(quadrant) {
        console.log("tl : " + quadrant.top_left)
        quadrant.top_left[0] = quadrant.top_left[0] - maxBBox.tl[0]
        quadrant.top_left[1] = quadrant.top_left[1] - maxBBox.br[1]
        console.log("normal tl : " + quadrant.top_left)
        if (quadrant.data === "empty") {
            return;
        }
        quadrant.center_of_mass[0] = quadrant.center_of_mass[0] - maxBBox.tl[0]
        quadrant.center_of_mass[1] = quadrant.center_of_mass[1] - maxBBox.br[1]
        if ("body" in quadrant.data) {
            quadrant.data.body.position[0] = quadrant.data.body.position[0] - maxBBox.tl[0]
            quadrant.data.body.position[1] = quadrant.data.body.position[1] - maxBBox.br[1]
        }

        if ("subquadrants" in quadrant.data) {
            navigateQuadtree(quadrant.data.subquadrants.nw);
            navigateQuadtree(quadrant.data.subquadrants.ne);
            navigateQuadtree(quadrant.data.subquadrants.se);
            navigateQuadtree(quadrant.data.subquadrants.sw);
        }

    }

    console.log(data[idx])
    navigateQuadtree(data[idx])

    return data[idx]
}

async function readFile(node) {
    const file = node.files[0];
    const obj = await fileToJSON(file);
    globalQuadtreeStepsData = obj['data'].map((s) => (s.quadtree))
    console.log("INITIAL GLOBAL DATA", globalQuadtreeStepsData)
    totalSteps = globalQuadtreeStepsData.length;

    // get list of bBoxes
    let bBoxes = await getBBoxes(obj['data'])

    // find max bBox
    maxBBox = getMaxBBox(bBoxes)
    console.log("max bBox " + maxBBox.tl + " - " + maxBBox.br)

    let q = normalizeQuadtree(currentStep, maxBBox)

    console.log("after normalization = (" + currentStep + "/" + totalSteps + ") ", q)

    console.log(globalQuadtreeStepsData)

    drawQuadtree(q)
    //drawQuadtree(obj);

}

function nextStep() {
    if (currentStep < (totalSteps - 1)) {
        currentStep++;
        console.log("rendering step " + currentStep)
        let q = normalizeQuadtree(currentStep, maxBBox)
        drawQuadtree(q)
    }
}

function prevStep() {
    if (currentStep > 1) {
        currentStep--;
        let q = normalizeQuadtree(currentStep, maxBBox)
        drawQuadtree(q)
    }
}

function drawQuadtree(quadtree) {
    const SVG_SIZE = parseInt(document.getElementById("svg-canvas").getAttribute("width"));
    let scenarioSize = quadtree.length;
    let ratio = SVG_SIZE / scenarioSize;

    document.getElementById("welcome-wrapper").setAttribute("display", "none");
    document.getElementById("welcome-wrapper").setAttribute("style", "display: none;");
    clearSvg();

    function navigateQuadtree(quadrant) {
        appendSquare(
            quadrant.top_left[0] * ratio, (scenarioSize - quadrant.top_left[1]) * ratio,
            quadrant.length * ratio, quadrant.length / scenarioSize,
            "subquad_x" + quadrant.top_left[0] + "_y" + quadrant.top_left[1] + "_l" + quadrant.length
        );

        if (quadrant.data === "empty"){
            return;
        }

        if("body" in quadrant.data){
            appendCircle(quadrant.data.body.position[0] * ratio, (scenarioSize - quadrant.data.body.position[1]) * ratio, "quadBody_x" + quadrant.data.body.position[0] + "_y" + quadrant.data.body.position[1]);
        }

        if("subquadrants" in quadrant.data){
            navigateQuadtree(quadrant.data.subquadrants.nw);
            navigateQuadtree(quadrant.data.subquadrants.ne);
            navigateQuadtree(quadrant.data.subquadrants.se);
            navigateQuadtree(quadrant.data.subquadrants.sw);
        }

    }

    navigateQuadtree(quadtree);
}