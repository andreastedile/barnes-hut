var ROOT_MinX;
var ROOT_MinY;
var ROOT_Width;
var ROOT_Height;

var SLEEP_Time = 0.2;
var USEMAX_Bbox = true;

var SVG_Width;
var SVG_Height;

function removeHeadings() {
    console.log("Remove headings");
    d3.select("#welcome-wrapper").remove();
    d3.select("#title").remove();
}

function showOptions() {
    document.getElementById("options").style.display = "block";
}

function setupMaxBbox(rootBbox) {
    const rootMinX = rootBbox.bottomLeft.x;
    const rootMinY = rootBbox.bottomLeft.y;
    const rootMaxX = rootBbox.topRight.x;
    const rootMaxY = rootBbox.topRight.y;
    const rootWidth = Math.abs(rootMaxX - rootMinX);
    const rootHeight = Math.abs(rootMaxY - rootMinY);
    ROOT_MinX = rootMinX;
    ROOT_MinY = rootMinY;
    ROOT_Width = rootWidth;
    ROOT_Height = rootHeight;
}

function setupSVGSize() {
    SVG_Width = window.innerWidth;
    SVG_Height = window.innerHeight;
}

/**
 * Produces the SVG from a quadtree
 */
function quadtreeToSvg(quadtree) {
    const svg = d3.create("svg");

    // get the viewbox size based on USEMAX flag
    let rootX = (USEMAX_Bbox) ? ROOT_MinX : quadtree.boundingBox.bottomLeft.x;
    let rootY = (USEMAX_Bbox) ? ROOT_MinY : quadtree.boundingBox.bottomLeft.y;
    let rootW = (USEMAX_Bbox) ? ROOT_Width : quadtree.boundingBox.topRight.x - quadtree.boundingBox.bottomLeft.x;
    let rootH = (USEMAX_Bbox) ? ROOT_Height : quadtree.boundingBox.topRight.y - quadtree.boundingBox.bottomLeft.y;

    // https://developer.mozilla.org/en-US/docs/Web/SVG/Attribute/viewBox
    svg.attr("viewBox", `${rootX} ${rootY} ${rootW} ${rootH}`)
        .attr("width", SVG_Width)
        .attr("height", SVG_Height);

    svg.append("rect")
        .attr("x", rootX)
        .attr("y", rootY)
        .attr("width", rootW)
        .attr("height", rootH)
        .style("stroke-width", "1")
        .style("stroke", "blue")
        .style("opacity", "0.05")
        .style("fill", "none")
        .style("vector-effect", "non-scaling-stroke")

    const nodes_to_process = [quadtree];
    while (nodes_to_process.length > 0) {
        const node = nodes_to_process.shift();

        const nodeBbox = node.boundingBox;
        const nodeMinX = nodeBbox.bottomLeft.x;
        const nodeMinY = nodeBbox.bottomLeft.y;
        const nodeMaxX = nodeBbox.topRight.x;
        const nodeMaxY = nodeBbox.topRight.y;
        const nodeWidth = Math.abs(nodeMaxX - nodeMinX);
        const nodeHeight = Math.abs(nodeMaxY - nodeMinY);

        const LINE_THICKNESS = 0.5;
        svg.append("rect")
            .attr("x", nodeMinX)
            .attr("y", nodeMinY)
            .attr("width", nodeWidth)
            .attr("height", nodeHeight)
            .style("stroke-width", LINE_THICKNESS)
            .style("stroke", "grey")
            .style("opacity", "0.5")
            .style("fill", "none")
            .style("vector-effect", "non-scaling-stroke")

        if (node.fork) {
            nodes_to_process.push(node.fork.nw, node.fork.ne, node.fork.se, node.fork.sw);
        } else if (node.leaf.body) {
            body = node.leaf.body;
            const CIRCLE_RADIUS = rootW / 300;//(ROOT_Width / 100) * body.mass;
            svg.append("circle")
                .style("vector-effect", "non-scaling-stroke")
                .attr("cx", body.position.x)
                .attr("cy", body.position.y)
                .attr("r", CIRCLE_RADIUS);
        }
    }

    return svg;
}

/**
 * Parses a JSON file containing the simulation data into a JavaScript object.
 * @param {File} file
 * @return {Promise<Object>} JavaScript object containing the parsed simulation data.
 */
async function parseJSON(file) {
    return new Promise((resolve, reject) => {
        const fileReader = new FileReader();
        fileReader.readAsText(file);
        fileReader.onload = event => {
            /** @type {string} */
            const result = event.target.result;
            const parsed = JSON.parse(result);
            resolve(parsed);
        }
        fileReader.onerror = error => reject(error);
    });
}

const sleep = (secs) => new Promise((res) => {
    console.log("sleeping... " + secs);
    setTimeout(res, secs * 1000)
});

async function loadSimulation(input) {
    const file = input.files[0];
    const json = await parseJSON(file);
    const nSteps = json.nSteps;

    // The quadtree of the first step consists only of a one root node without any bodies.
    if (nSteps === 0) return;

    setupMaxBbox(json.maxBoundingBox);
    setupSVGSize();

    for (let i = 1; i < nSteps; i++) {
        console.log("Step " + i);

        // Remove the old quadtree svg
        d3.select("#svg-wrapper").selectAll("*").remove();

        const quadtree = json.simulationSteps[i].quadtree;

        // Set the new quadtree svg
        d3.select("#svg-wrapper").append(() => quadtreeToSvg(quadtree).node());

        if (SLEEP_Time > 0) {
            await sleep(SLEEP_Time);
        }
    }
}

function toggleViewbox(input) {
    console.log("toggling viewbox!", input.checked);
    if (input.checked) {
        USEMAX_Bbox = true;
    } else {
        USEMAX_Bbox = false;
    }
}

function updateSleep(input) {
    if (input.value < 0) {
        input.value = 0;
    } else if (input.value > 500) {
        input.value = 500;
    }
    console.log("updating sleep!", input.value);
    SLEEP_Time = input.value / 1000;
}