console.log("loaded animation.js");

async function readFile(node) {
    const file = node.files[0];
    const obj = await fileToJSON(file);
    console.log(obj);
    simulateParticlesAnimation();
}

function generateCircles() {
    for (let i = 0; i < N_STEPS; i++) {
        let circles = [];
        for (let j = 0; j < N_CIRCLES; j++) {
            // if it's the first step in history
            if (i === 0) {
                let cx = Math.floor(Math.random() * SVG_SIZE);
                let cy = Math.floor(Math.random() * SVG_SIZE);
                circles.push({idx: j, id: "particle_" + j, cx, cy});
            } else {
                let cx = circlesHistory[i - 1][j].cx + (Math.floor(Math.random() * 100) - 50);
                let cy = circlesHistory[i - 1][j].cy + (Math.floor(Math.random() * 100) - 50);
                circles.push({idx: j, id: "particle_" + j, cx, cy});
            }
        }
        circlesHistory.push(circles);
    }
    console.log(circlesHistory);
}

// I take the first step in history to set up the cirlces on screen
function appendCircles() {
    circlesHistory[0].map((circle) => {
        appendCircle(circle.cx, circle.cy, circle.id);
    });
}

// given the history I set up an array of animations, with the keyframes of each circle
function setUpCirclesAnimation() {
    circlesAnimation = circlesHistory[0].map((circle) => {
        let circleKeyFrames = [];
        // I retrieve all the steps of the circle from the history
        for (let i = 0; i < circlesHistory.length; i++) {
            circleKeyFrames.push({cx: circlesHistory[i][circle.idx].cx, cy: circlesHistory[i][circle.idx].cy});
        }
        return anime({
            targets: "svg#svg-canvas #" + circle.id,
            // Properties
            keyframes: circleKeyFrames,
            duration: N_STEPS * 500, //each step will last 500ms
            autoplay: false,
            easing: 'linear',
        });
    });
}

function startCirclesAnimation() {
    circlesAnimation.map((circleAnimation) => {
        circleAnimation.play();
    });
}

function simulateParticlesAnimation() {

    document.getElementById("welcome-wrapper").setAttribute("display", "none");
    document.getElementById("welcome-wrapper").setAttribute("style", "display: none;");
    clearSvg();
    circlesHistory = [];
    circlesAnimation = [];

    generateCircles();

    appendCircles();
    setUpCirclesAnimation();
    startCirclesAnimation();
}