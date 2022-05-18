console.log("loaded quadtree.js");

async function readFile(node) {
    const file = node.files[0];
    const obj = await fileToJSON(file);
    console.log(obj);
    drawQuadtree(obj);
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