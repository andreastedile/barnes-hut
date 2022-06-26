var ROOT_MinX;
var ROOT_MinY;
var ROOT_Width;
var ROOT_Height;

function removeHeadings() {
    console.log("Remove headings");
    d3.select("#welcome-wrapper").remove();
    d3.select("#title").remove();
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

/**
 * Produces the SVG from a quadtree
 */
function quadtreeToSvg(quadtree) {
    const svg = d3.create("svg");

    // https://developer.mozilla.org/en-US/docs/Web/SVG/Attribute/viewBox
    svg.attr("viewBox", `${ROOT_MinX} ${ROOT_MinY} ${ROOT_Width} ${ROOT_Height}`)
        .attr("width", window.innerWidth)
        .attr("height", window.innerHeight);

    svg.append("rect")
        .attr("x", ROOT_MinX)
        .attr("y", ROOT_MinY)
        .attr("width", ROOT_Width)
        .attr("height", ROOT_Height)
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
            const CIRCLE_RADIUS = (ROOT_Width / 100) * body.mass;
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

const sleep = (secs) => new Promise((res) => setTimeout(res, secs * 1000));

async function loadSimulation(input) {
    const file = input.files[0];
    const json = await parseJSON(file);
    const nSteps = json.nSteps;

    // The quadtree of the first step consists only of a one root node without any bodies.
    if (nSteps === 0) return;

    setupMaxBbox(json.maxBoundingBox);

    for (let i = 1; i < nSteps; i++) {
        console.log("Step " + i);

        // Remove the old quadtree svg
        d3.select("#svg-wrapper").selectAll("*").remove();

        const quadtree = json.simulationSteps[i].quadtree;

        // Set the new quadtree svg
        d3.select("#svg-wrapper").append(() => quadtreeToSvg(quadtree).node());

        await sleep(0.2);
    }
}