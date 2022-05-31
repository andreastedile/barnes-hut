console.log("loaded animation.js");

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

async function getParticlesSteps(data) {
    let particlesStep = await Promise.all(data.map(o => {
        return new Promise(async (resolve, reject) => {
            let p = await Promise.all(o.bodies.map(particle => {
                {
                    return new Promise((resolve1, reject1) => {
                        resolve1({
                            mass: particle.mass,
                            position: particle.position
                        })
                    })
                }
            }))
            resolve(p)
        })
    }))
    return particlesStep

}

async function getSvgNormalizedHistory(maxBBox, particlesStep) {
    const SVG_SIZE = parseInt(document.getElementById("svg-canvas").getAttribute("width"));
    const SCENARIO_SIZE = maxBBox.br[0] - maxBBox.tl[0]
    let ratio = SVG_SIZE / SCENARIO_SIZE;
    console.log("scenario size: " + SCENARIO_SIZE + " | svg size: " + SVG_SIZE + " | ratio: " + ratio)

    let normalizedParticlesHistory = await Promise.all(particlesStep[0].map((particle, idx) => {
        return new Promise(async (resolve, reject) => {
            let hist = {mass: particle.mass, positions: []}
            hist.positions = await Promise.all(particlesStep.map(step => {
                return new Promise((resolve1, reject1) => {
                    resolve1([(step[idx].position[0] - maxBBox.tl[0]) * ratio, SVG_SIZE - ((step[idx].position[1] - maxBBox.br[1]) * ratio)])
                })
            }))
            resolve(hist)
        })
    }));
    return normalizedParticlesHistory

}

async function printCircles(svgParticlesHist) {
    await Promise.all(svgParticlesHist.map((pHist, idx) => {
        return new Promise((resolve, reject) => {
            appendCircle(pHist.positions[0][0], pHist.positions[0][1], "c_" + idx)
            resolve()
        })
    }))

}

// given the history I set up an array of animations, with the keyframes of each circle
async function setUpCirclesAnimation(svgParticlesHist) {
    let circlesAnimation = await Promise.all(svgParticlesHist.map((pHist, idx) => {
        return new Promise(async (resolve, reject) => {
            let circleKeyFrames = await Promise.all(pHist.positions.map(p => {
                return new Promise((resolve1, reject1) => resolve1({cx: p[0], cy: p[1]}))
            }));
            resolve(anime({
                targets: "svg#svg-canvas #" + "c_" + idx,
                // Properties
                keyframes: circleKeyFrames,
                duration: N_STEPS * 125, //each step will last 500ms
                autoplay: false,
                easing: 'linear',
            }));
        })
    }));
    return circlesAnimation
}

async function startCirclesAnimation(circlesAnimation) {
    await Promise.all(circlesAnimation.map((circleAnimation) => {
        return new Promise((resolve, reject1) => {
            circleAnimation.play();
            resolve()
        })
    }))

}

async function simulateParticlesAnimation(svgParticlesHist) {
    await printCircles(svgParticlesHist)

    let circlesAnimation = await setUpCirclesAnimation(svgParticlesHist)

    setTimeout(async () => {
        await startCirclesAnimation(circlesAnimation)
    }, 1)

}

async function initialSetup(obj) {
    document.getElementById("welcome-wrapper").setAttribute("display", "none");
    document.getElementById("welcome-wrapper").setAttribute("style", "display: none;");
    clearSvg();

    console.log(obj);

    // get list of bBoxes
    let bBoxes = await getBBoxes(obj['data'])
    console.log(bBoxes)

    // find max bBox
    let maxBBox = await getMaxBBox(bBoxes)
    console.log("max bBox " + maxBBox.tl + " - " + maxBBox.br)

    //extract particles steps
    let particlesStep = await getParticlesSteps(obj['data'])
    console.log(particlesStep)

    //make normalized particle history
    let normalizedParticlesHistory = await getSvgNormalizedHistory(maxBBox, particlesStep)
    console.log(normalizedParticlesHistory)

    //simulateParticlesAnimation(normalizedParticlesHistory, maxBBox.br[0] - maxBBox.tl[0]);
    return normalizedParticlesHistory

}

async function readFile(node) {
    let obj = await fileToJSON(node.files[0])
    hist = await initialSetup(obj)
    setTimeout(async () => {
        await simulateParticlesAnimation(hist)
    }, 1)
}