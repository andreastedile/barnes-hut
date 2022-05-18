XMLNS = "http://www.w3.org/2000/svg";
N_CIRCLES = 200;
N_STEPS = 100;
CIRCLE_STYLE = {
    stroke: "yellow",
    strokeWidth: 1,
    fill: "red",
    opacity: 1.0,
    r: 4
};
SQUARE_STYLE = {
    stroke: "blue",
    strokeWidth: 1.5,
    fill: "none"
};
SVG_SIZE = 700;

circlesHistory = [];
circlesAnimation = [];


function clearContent(node) {
    console.log("clear content");
    node.value = null;
}

async function fileToJSON(file) {
    return new Promise((resolve, reject) => {
        const fileReader = new FileReader()
        fileReader.onload = event => resolve(JSON.parse(event.target.result))
        fileReader.onerror = error => reject(error)
        fileReader.readAsText(file)
    })
}

function clearSvg() {
    let svgCanvas = document.getElementById("svg-canvas");
    while (svgCanvas.firstChild) {
        svgCanvas.removeChild(svgCanvas.lastChild);
    }

    console.log("svg has been cleared");
    document.getElementById("svg-wrapper").setAttribute("display", "block");
    document.getElementById("svg-wrapper").setAttribute("style", "display: block;");
}

// appends a circle element into the SVG field
function appendCircle(cx, cy, id) {
    let circle = document.createElementNS(XMLNS, "circle");

    circle.setAttributeNS(null, 'stroke', CIRCLE_STYLE.stroke);
    circle.setAttributeNS(null, 'id', id);
    circle.setAttributeNS(null, 'stroke-width', CIRCLE_STYLE.strokeWidth);
    circle.setAttributeNS(null, 'cx', cx);
    circle.setAttributeNS(null, 'cy', cy);
    circle.setAttributeNS(null, 'r', CIRCLE_STYLE.r);
    circle.setAttributeNS(null, 'fill', CIRCLE_STYLE.fill);
    circle.setAttributeNS(null, 'opacity', CIRCLE_STYLE.opacity);

    document.getElementById("svg-canvas").appendChild(circle);
}

// appends a square element into the SVG field
function appendSquare(x, y, l, o, id) {
    let rectangle = document.createElementNS(XMLNS, "rect");

    rectangle.setAttributeNS(null, 'stroke', SQUARE_STYLE.stroke);
    rectangle.setAttributeNS(null, 'id', id);
    rectangle.setAttributeNS(null, 'stroke-width', SQUARE_STYLE.strokeWidth);
    rectangle.setAttributeNS(null, 'x', x);
    rectangle.setAttributeNS(null, 'y', y);
    rectangle.setAttributeNS(null, 'width', l);
    rectangle.setAttributeNS(null, 'height', l);
    rectangle.setAttributeNS(null, 'fill', SQUARE_STYLE.fill);
    rectangle.setAttributeNS(null, 'opacity', o);

    document.getElementById("svg-canvas").appendChild(rectangle);
}