function removeHeadings() {
    console.log("Remove headings");
    d3.select("#welcome-wrapper").remove();
    d3.select("#title").remove();
}

/**
 * Produces the SVG from a quadtree
 */
function quadtreeToSvg(quadtree) {
    const svg = d3.create("svg");

    // https://developer.mozilla.org/en-US/docs/Web/SVG/Attribute/viewBox
    const bbox = quadtree.boundingBox;
    const bl = bbox.bottomLeft;
    const tr = bbox.topRight;
    const minX = bl.x;
    const minY = bl.y;
    const maxX = tr.x;
    const maxY = tr.y;
    const bboxWidth = Math.abs(maxX - minX);
    const bboxHeight = Math.abs(maxY - minY);
    svg.attr("viewBox", `${minX} ${minY} ${bboxWidth} ${bboxHeight}`)
        .attr("width", window.innerHeight)
        .attr("height", window.innerHeight);

    const quadtrees = [quadtree];
    while (quadtrees.length > 0) {
        const qt = quadtrees.shift();

        const rectBbox = qt.boundingBox;
        const rectBl = rectBbox.bottomLeft;
        const rectTl = rectBbox.topRight;
        const rectMinX = rectBl.x;
        const rectMinY = rectBl.y;
        const rectMaxX = rectTl.x;
        const rectMaxY = rectTl.y;
        const rectWidth = Math.abs(rectMaxX - rectMinX);
        const rectHeight = Math.abs(rectMaxY - rectMinY);

        const STROKE_WIDTH = 0.5;
        svg.append("rect")
            .attr("x", rectMinX)
            .attr("y", rectMinY)
            .attr("width", rectWidth)
            .attr("height", rectHeight)
            .style("stroke-width", STROKE_WIDTH)
            .style("stroke", "grey")
            .style("opacity", "0.5")
            .style("fill", "none")
            .style("vector-effect", "non-scaling-stroke")

        if (qt.fork) {
            quadtrees.push(qt.fork.nw, qt.fork.ne, qt.fork.se, qt.fork.sw);
        } else if (qt.leaf.body) {
            body = qt.leaf.body;
            let CIRCLE_RADIUS = bboxWidth / 100;
            CIRCLE_RADIUS *= body.mass;
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

    for (let i = 1; i < nSteps; i++) {
        console.log("Step " + i);

        // Remove the old quadtree svg
        d3.select("#svg-wrapper").selectAll("*").remove();

        const quadtree = json.simulationSteps[i].quadtree;

        // Set the new quadtree svg
        d3.select("#svg-wrapper").append(() => quadtreeToSvg(quadtree).node());

        await sleep(0.5);
    }
}