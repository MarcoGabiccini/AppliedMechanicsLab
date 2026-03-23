var Example = Example || {};

Example.strandbeast_leg = function() {
    var Engine = Matter.Engine,
        Events = Matter.Events,
        Render = Matter.Render,
        Runner = Matter.Runner,
        Body = Matter.Body,
        Composite = Matter.Composite,
        Bodies = Matter.Bodies,
        Vector = Matter.Vector;

    // create engine
    // Matter is used here as a scene/update/render loop container.
    // The four-bar closure itself is solved analytically below, not by Matter constraints.
    var engine = Engine.create(),
        world = engine.world;

    engine.gravity.y = 0;

    // create renderer
    var render = Render.create({
        element: document.body,
        engine: engine,
        options: {
            width: window.innerWidth,
            height: window.innerHeight,
            wireframes: false,
            background: '#1a1a2e'
        }
    });

    render.canvas.style.display = 'block';
    render.canvas.style.position = 'fixed';
    render.canvas.style.left = '0';
    render.canvas.style.top = '0';

    Render.run(render);

    // create runner
    var runner = Runner.create();
    Runner.run(runner, engine);

    // ================================================================
    // Coordinate system
    // ================================================================
    //
    //  User coords:  O1 = (0,0), x right, y UP
    //  Screen coords: sx = ux * S + TX,  sy = -uy * S + TY
    //
    //  Four-bar loop:  O2 --(m)--> A --(j)--> B --(b)--> O1
    //  Ground link:    O1 -- O2  (fixed)
    //  Crank m driven at constant angular velocity around O2
    //
    //  Important: this example is kinematic, not dynamic.
    //  We do not ask Matter.js to close the loop with constraints.
    //  Instead, at every update we solve the closed-chain geometry and
    //  then place all rendered bodies exactly on the solved poses.
    //

    // link lengths (user units)
    var LINK_m = 15.0,   // crank (O2 -> A)
        LINK_j = 50.0,   // coupler (A -> B)
        LINK_b = 41.5,   // rocker (O1 -> B)
        LINK_k = 61.9,   // coupler (A -> C)
        LINK_c = 39.3,   // rocker (O1 -> C)
        LINK_e = 55.8,   // side (B -> D)
        LINK_d = 40.1,   // side (O1 -> D)
        LINK_g = 36.7,   // link (C -> E)
        LINK_f = 39.4,   // link (D -> E)
        LINK_h = 67.5,   // side (E -> F)
        LINK_i = 49.0,   // side (C -> F)
        LINK_a = 38.0,   // O2.x relative to O1
        LINK_l = 7.8;    // O2.y relative to O1

    // scale & translation: user coords -> screen coords
    var S = 4;
    // Horizontal global offset expressed in user units.
    var OFFSET_X_UNITS = 100;
    var BASE_TX = 300;
    var TX = BASE_TX + OFFSET_X_UNITS * S, TY = 350;

    function toScreen(ux, uy) {
        return { x: ux * S + TX, y: -uy * S + TY };
    }

    function angleOfLink(p1, p2) {
        return Math.atan2(p2.y - p1.y, p2.x - p1.x);
    }

    function triangleCentroid(p1, p2, p3) {
        return {
            x: (p1.x + p2.x + p3.x) / 3,
            y: (p1.y + p2.y + p3.y) / 3
        };
    }

    // ================================================================
    // Circle-circle intersection in USER coords
    // Returns the intersection point of circles C(p1,r1) and C(p2,r2).
    // sign = +1 or -1 selects which of the two solutions.
    // ================================================================
    function cci(p1, r1, p2, r2, sign) {
        var dx = p2.x - p1.x, dy = p2.y - p1.y;
        var d = Math.sqrt(dx * dx + dy * dy);
        var a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
        var h = Math.sqrt(Math.max(0, r1 * r1 - a * a));
        var mx = p1.x + a * dx / d;
        var my = p1.y + a * dy / d;
        return {
            x: mx + sign * h * dy / d,
            y: my - sign * h * dx / d
        };
    }

    // ================================================================
    // Initial closed configuration (user coords)
    // ================================================================
    // Fixed pivots (user coords)
    var O1u = { x: 0, y: 0 };
    var O2u = { x: LINK_a, y: LINK_l };

    function solveClosedConfiguration(thetaM, branchSignB, branchSignC, branchSignD, branchSignE, branchSignF) {
        var Au = {
            x: O2u.x + LINK_m * Math.cos(thetaM),
            y: O2u.y + LINK_m * Math.sin(thetaM)
        };

        // Closed-chain compatibility:
        // once theta_m is fixed, point A is known.
        // Then B must simultaneously lie on:
        //   1) the circle centered at A with radius j
        //   2) the circle centered at O1 with radius b
        // branchSign selects the assembly mode (upper or lower intersection).
        var Bu = cci(Au, LINK_j, O1u, LINK_b, branchSignB);
        var Cu = cci(Au, LINK_k, O1u, LINK_c, branchSignC);
        var Du = cci(Bu, LINK_e, O1u, LINK_d, branchSignD);
        var Eu = cci(Cu, LINK_g, Du, LINK_f, branchSignE);
        var Fu = cci(Eu, LINK_h, Cu, LINK_i, branchSignF);
        var thetaJ = angleOfLink(Au, Bu);
        var thetaB = angleOfLink(O1u, Bu);
        var thetaK = angleOfLink(Au, Cu);
        var thetaC = angleOfLink(O1u, Cu);
        var thetaG = angleOfLink(Cu, Eu);
        var thetaF = angleOfLink(Du, Eu);

        return {
            A: Au,
            B: Bu,
            C: Cu,
            D: Du,
            E: Eu,
            F: Fu,
            theta_m: thetaM,
            theta_j: thetaJ,
            theta_b: thetaB,
            theta_k: thetaK,
            theta_c: thetaC,
            theta_g: thetaG,
            theta_f: thetaF
        };
    }

    // Start from a consistent assembly mode of the closed chain.
    // For loop O2-A-B-O1, branchSignB = +1 selects the upper intersection.
    // For loop O2-A-C-O1, branchSignC = -1 selects the lower intersection.
    // For triangle O1-B-D, branchSignD = +1 selects the upper-left intersection.
    // For loop C-E-D, branchSignE = -1 selects the alternative solution with y_E < 0.
    // For triangle E-C-F, branchSignF = +1 selects the lower-left solution.
    // With the current dimensions this gives:
    // C0 ~= (11.05, -37.72), D0 ~= (-36.79, 15.94), E0 ~= (-2.61, -3.65).
    var initialConfig = solveClosedConfiguration(0, 1, -1, 1, -1, 1);
    var theta_m0 = initialConfig.theta_m;
    var theta_j0 = initialConfig.theta_j;
    var theta_b0 = initialConfig.theta_b;
    var theta_k0 = initialConfig.theta_k;
    var theta_c0 = initialConfig.theta_c;
    var theta_g0 = initialConfig.theta_g;
    var theta_f0 = initialConfig.theta_f;
    var A0u = initialConfig.A;
    var B0u = initialConfig.B;
    var C0u = initialConfig.C;
    var D0u = initialConfig.D;
    var E0u = initialConfig.E;
    var F0u = initialConfig.F;

    // Convert to screen coords
    var O1 = toScreen(O1u.x, O1u.y);
    var O2 = toScreen(O2u.x, O2u.y);
    var A0 = toScreen(A0u.x, A0u.y);
    var B0 = toScreen(B0u.x, B0u.y);
    var C0 = toScreen(C0u.x, C0u.y);
    var D0 = toScreen(D0u.x, D0u.y);
    var E0 = toScreen(E0u.x, E0u.y);
    var F0 = toScreen(F0u.x, F0u.y);

    // ================================================================
    // Create bodies
    // ================================================================
    // These bodies are visual/interactive carriers only.
    // They are static because the mechanism motion is prescribed by the
    // kinematic solver, not produced by forces or constraint impulses.
    //
    // makeLink: creates a rectangle body whose endpoints exactly match p1, p2.
    // The body length = distance(p1, p2), centered at midpoint, rotated to match.
    var group = Body.nextGroup(true),
        w = 12;

    function makeLink(p1, p2, color) {
        var dx = p2.x - p1.x, dy = p2.y - p1.y;
        var len = Math.sqrt(dx * dx + dy * dy);
        var cx = (p1.x + p2.x) / 2, cy = (p1.y + p2.y) / 2;
        var body = Bodies.rectangle(cx, cy, len, w, {
            isStatic: true,
            collisionFilter: { group: group },
            frictionAir: 0, friction: 0, restitution: 0,
            render: { fillStyle: color, strokeStyle: '#222', lineWidth: 1 }
        });
        Body.setAngle(body, Math.atan2(dy, dx));
        body._halfLen = len / 2;
        return body;
    }

    var crankBody    = makeLink(O2, A0, '#e6b800');   // crank m (yellow)
    var couplerBBody = makeLink(A0, B0, '#33dd55');   // coupler j (green)
    var rockerBBody  = makeLink(O1, B0, '#ff8800');   // rocker b (orange)
    var couplerCBody = makeLink(A0, C0, '#4cc9f0');   // coupler k (cyan)
    var rockerCBody  = makeLink(O1, C0, '#b5179e');   // rocker c (magenta)
    var couplerEBody = makeLink(C0, E0, '#f72585');   // link g (pink)
    var rockerEBody  = makeLink(D0, E0, '#7209b7');   // link f (violet)

    // For a rigid arbitrary planar shape, Matter's recommended body API is
    // Bodies.fromVertices(...). We use it here for the rigid triangle O1-B-D.
    var triangleCenter0 = triangleCentroid(O1, B0, D0);
    var triangleEdgeAngle0 = angleOfLink(O1, B0);
    var triangleBody = Bodies.fromVertices(triangleCenter0.x, triangleCenter0.y, [
        { x: O1.x, y: O1.y },
        { x: B0.x, y: B0.y },
        { x: D0.x, y: D0.y }
    ], {
        isStatic: true,
        collisionFilter: { group: group },
        frictionAir: 0, friction: 0, restitution: 0,
        render: {
            fillStyle: 'rgba(168,181,255,0.18)',
            strokeStyle: '#a8b5ff',
            lineWidth: 2
        }
    });

    var triangleECFCenter0 = triangleCentroid(E0, C0, F0);
    var triangleECFEdgeAngle0 = angleOfLink(C0, E0);
    var triangleECFBody = Bodies.fromVertices(triangleECFCenter0.x, triangleECFCenter0.y, [
        { x: E0.x, y: E0.y },
        { x: C0.x, y: C0.y },
        { x: F0.x, y: F0.y }
    ], {
        isStatic: true,
        collisionFilter: { group: group },
        frictionAir: 0, friction: 0, restitution: 0,
        render: {
            fillStyle: 'rgba(255, 190, 120, 0.16)',
            strokeStyle: '#ffd39a',
            lineWidth: 2
        }
    });

    Composite.add(world, [
        crankBody,
        couplerBBody,
        rockerBBody,
        couplerCBody,
        rockerCBody,
        couplerEBody,
        rockerEBody,
        triangleBody,
        triangleECFBody
    ]);

    function setLinkPose(body, p1, p2, updateVelocity) {
        var dx = p2.x - p1.x, dy = p2.y - p1.y;
        Body.setPosition(body, {
            x: (p1.x + p2.x) / 2,
            y: (p1.y + p2.y) / 2
        }, updateVelocity);
        Body.setAngle(body, Math.atan2(dy, dx), updateVelocity);
    }

    function syncMechanism(config, updateVelocity) {
        // Convert the exact kinematic solution into rendered body poses.
        // This guarantees:
        // - crank endpoints at O2 and A
        // - coupler AB endpoints at A and B
        // - rocker O1B endpoints at O1 and B
        // - coupler AC endpoints at A and C
        // - rocker O1C endpoints at O1 and C
        // - rigid triangle vertices at O1, B and D
        // - link g endpoints at C and E
        // - link f endpoints at D and E
        // - rigid triangle vertices at E, C and F
        var Au = config.A;
        var Bu = config.B;
        var Cu = config.C;
        var Du = config.D;
        var Eu = config.E;
        var Fu = config.F;
        var As = toScreen(Au.x, Au.y);
        var Bs = toScreen(Bu.x, Bu.y);
        var Cs = toScreen(Cu.x, Cu.y);
        var Ds = toScreen(Du.x, Du.y);
        var Es = toScreen(Eu.x, Eu.y);
        var Fs = toScreen(Fu.x, Fu.y);
        var triangleCenter = triangleCentroid(O1, Bs, Ds);
        var triangleAngle = angleOfLink(O1, Bs) - triangleEdgeAngle0;
        var triangleECFCenter = triangleCentroid(Es, Cs, Fs);
        var triangleECFAngle = angleOfLink(Cs, Es) - triangleECFEdgeAngle0;

        setLinkPose(crankBody, O2, As, updateVelocity);
        setLinkPose(couplerBBody, As, Bs, updateVelocity);
        setLinkPose(rockerBBody, O1, Bs, updateVelocity);
        setLinkPose(couplerCBody, As, Cs, updateVelocity);
        setLinkPose(rockerCBody, O1, Cs, updateVelocity);
        setLinkPose(couplerEBody, Cs, Es, updateVelocity);
        setLinkPose(rockerEBody, Ds, Es, updateVelocity);
        Body.setPosition(triangleBody, triangleCenter, updateVelocity);
        Body.setAngle(triangleBody, triangleAngle, updateVelocity);
        Body.setPosition(triangleECFBody, triangleECFCenter, updateVelocity);
        Body.setAngle(triangleECFBody, triangleECFAngle, updateVelocity);
    }

    syncMechanism(initialConfig, false);

    // ================================================================
    // Drive crank with exact kinematic closure
    // ================================================================
    // In Matter.js positive angle is CW (y-down).
    // User wants CCW in their coords (y-up), so positive crankSpeed
    // in Matter.js corresponds to CW on screen = CCW in user coords.
    // We negate so that positive slider = CCW in user coords.
    //
    // The update is purely kinematic:
    // 1) advance theta_m
    // 2) solve the closed-chain geometry
    // 3) place the link bodies on the solved configuration
    //
    // No Matter.Constraint objects are used for the mechanism.
    var crankSpeed = -0.03;
    var assemblySignB = 1;
    var assemblySignC = -1;
    var assemblySignD = 1;
    var assemblySignE = -1;
    var assemblySignF = 1;
    var isRunning = true;
    var theta_m = theta_m0;
    var currentConfig = initialConfig;

    Events.on(engine, 'beforeUpdate', function(event) {
        if (!isRunning) {
            return;
        }

        var timeScale = event.delta / (1000 / 60);

        theta_m += -crankSpeed * timeScale;
        currentConfig = solveClosedConfiguration(theta_m, assemblySignB, assemblySignC, assemblySignD, assemblySignE, assemblySignF);
        syncMechanism(currentConfig, false);
    });

    // ================================================================
    // UI: speed slider
    // ================================================================
    var panel = document.createElement('div');
    panel.style.cssText = 'position:absolute;top:10px;right:10px;color:#ccc;font:13px monospace;' +
        'background:rgba(0,0,0,0.65);padding:12px;border-radius:8px;user-select:none;z-index:10;';
    panel.innerHTML =
        '<b style="color:#fff">Strandbeast Leg Linkage</b><br><br>' +
        '<div style="margin-bottom:8px;color:#aaa">' +
            'theta_m0=' + theta_m0.toFixed(3) + '<br>' +
            'theta_j0=' + theta_j0.toFixed(3) + '<br>' +
            'theta_b0=' + theta_b0.toFixed(3) + '<br>' +
            'theta_k0=' + theta_k0.toFixed(3) + '<br>' +
            'theta_c0=' + theta_c0.toFixed(3) + '<br>' +
            'theta_g0=' + theta_g0.toFixed(3) + '<br>' +
            'theta_f0=' + theta_f0.toFixed(3) +
        '</div>' +
        '<label>Velocita: <input type="range" id="fbSpeed" min="-100" max="100" value="30" style="width:140px;vertical-align:middle"></label>' +
        '<span id="fbSpeedVal" style="margin-left:6px">0.030</span>' +
        '<div style="margin-top:10px;display:flex;gap:8px">' +
            '<button id="fbToggle" style="flex:1;background:#2d6a4f;color:#fff;border:0;border-radius:4px;padding:6px 8px;cursor:pointer">Stop</button>' +
            '<button id="fbReset" style="flex:1;background:#8d6e00;color:#fff;border:0;border-radius:4px;padding:6px 8px;cursor:pointer">Reset</button>' +
        '</div>';
    document.body.appendChild(panel);

    document.getElementById('fbSpeed').addEventListener('input', function() {
        crankSpeed = -this.value / 1000;
        document.getElementById('fbSpeedVal').textContent = (this.value / 1000).toFixed(3);
    });

    function resetMechanism() {
        theta_m = theta_m0;
        currentConfig = solveClosedConfiguration(theta_m, assemblySignB, assemblySignC, assemblySignD, assemblySignE, assemblySignF);
        syncMechanism(currentConfig, false);
        trailB.length = 0;
        trailC.length = 0;
        trailF.length = 0;
    }

    document.getElementById('fbToggle').addEventListener('click', function() {
        isRunning = !isRunning;
        this.textContent = isRunning ? 'Stop' : 'Play';
        this.style.background = isRunning ? '#2d6a4f' : '#1d3557';
    });

    document.getElementById('fbReset').addEventListener('click', function() {
        resetMechanism();
    });

    // ================================================================
    // Rendering: labels, pivots, trail
    // ================================================================
    var trailB = [];
    var trailC = [];
    var trailF = [];

    Events.on(render, 'afterRender', function() {
        var ctx = render.context;

        // current joint positions from the exact kinematic state
        var jointA = toScreen(currentConfig.A.x, currentConfig.A.y);
        var jointB = toScreen(currentConfig.B.x, currentConfig.B.y);
        var jointC = toScreen(currentConfig.C.x, currentConfig.C.y);
        var jointD = toScreen(currentConfig.D.x, currentConfig.D.y);
        var jointE = toScreen(currentConfig.E.x, currentConfig.E.y);
        var jointF = toScreen(currentConfig.F.x, currentConfig.F.y);

        // trails on joints B, C and F
        if (isRunning) {
            trailB.unshift(Vector.clone(jointB));
            trailC.unshift(Vector.clone(jointC));
            trailF.unshift(Vector.clone(jointF));
            if (trailB.length > 2000) trailB.pop();
            if (trailC.length > 2000) trailC.pop();
            if (trailF.length > 2000) trailF.pop();
        }

        Render.startViewTransform(render);
        ctx.globalAlpha = 0.7;
        for (var i = 0; i < trailB.length; i++) {
            var age = i / trailB.length;
            ctx.fillStyle = 'rgba(255,80,80,' + (1 - age * 0.8) + ')';
            ctx.fillRect(trailB[i].x, trailB[i].y, 2, 2);
        }
        for (var j = 0; j < trailC.length; j++) {
            var ageC = j / trailC.length;
            ctx.fillStyle = 'rgba(80,200,255,' + (1 - ageC * 0.8) + ')';
            ctx.fillRect(trailC[j].x, trailC[j].y, 2, 2);
        }
        for (var k = 0; k < trailF.length; k++) {
            var ageF = k / trailF.length;
            ctx.fillStyle = 'rgba(255,40,40,' + (1 - ageF * 0.8) + ')';
            ctx.fillRect(trailF[k].x, trailF[k].y, 2, 2);
        }
        ctx.globalAlpha = 1;
        Render.endViewTransform(render);

        // fixed pivots
        function drawPivot(p, label) {
            ctx.beginPath(); ctx.arc(p.x, p.y, 7, 0, Math.PI * 2);
            ctx.fillStyle = '#cc2222'; ctx.fill();
            ctx.beginPath(); ctx.arc(p.x, p.y, 4, 0, Math.PI * 2);
            ctx.fillStyle = '#1a1a2e'; ctx.fill();
            ctx.beginPath(); ctx.arc(p.x, p.y, 2, 0, Math.PI * 2);
            ctx.fillStyle = '#cc2222'; ctx.fill();
            ctx.font = 'bold 14px monospace';
            ctx.fillStyle = '#ff6666';
            ctx.fillText(label, p.x + 10, p.y - 10);
        }

        // mobile joints
        function drawJoint(p, label) {
            ctx.beginPath(); ctx.arc(p.x, p.y, 5, 0, Math.PI * 2);
            ctx.fillStyle = '#ffffff'; ctx.fill();
            ctx.strokeStyle = '#333'; ctx.lineWidth = 1; ctx.stroke();
            ctx.font = 'bold 13px monospace';
            ctx.fillStyle = '#ffaa88';
            ctx.fillText(label, p.x + 8, p.y - 8);
        }

        drawPivot(O1, 'O\u2081');
        drawPivot(O2, 'O\u2082');
        drawJoint(jointA, 'A');
        drawJoint(jointB, 'B');
        drawJoint(jointC, 'C');
        drawJoint(jointD, 'D');
        drawJoint(jointE, 'E');
        drawJoint(jointF, 'F');

        // crank circle (dashed)
        ctx.beginPath();
        ctx.arc(O2.x, O2.y, LINK_m * S, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(255,255,255,0.12)';
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]); ctx.stroke(); ctx.setLineDash([]);

        // link labels
        ctx.font = '11px monospace';
        ctx.fillStyle = '#888';
        ctx.fillText('m=' + LINK_m, crankBody.position.x - 15, crankBody.position.y - 10);
        ctx.fillText('j=' + LINK_j, couplerBBody.position.x - 15, couplerBBody.position.y - 10);
        ctx.fillText('b=' + LINK_b, rockerBBody.position.x - 15, rockerBBody.position.y - 10);
        ctx.fillText('k=' + LINK_k, couplerCBody.position.x - 15, couplerCBody.position.y - 10);
        ctx.fillText('c=' + LINK_c, rockerCBody.position.x - 15, rockerCBody.position.y - 10);
        ctx.fillText('e=' + LINK_e, (jointB.x + jointD.x) / 2 - 15, (jointB.y + jointD.y) / 2 - 10);
        ctx.fillText('d=' + LINK_d, (O1.x + jointD.x) / 2 - 15, (O1.y + jointD.y) / 2 - 10);
        ctx.fillText('g=' + LINK_g, (jointC.x + jointE.x) / 2 - 15, (jointC.y + jointE.y) / 2 - 10);
        ctx.fillText('f=' + LINK_f, (jointD.x + jointE.x) / 2 - 15, (jointD.y + jointE.y) / 2 - 10);
        ctx.fillText('h=' + LINK_h, (jointE.x + jointF.x) / 2 - 15, (jointE.y + jointF.y) / 2 - 10);
        ctx.fillText('i=' + LINK_i, (jointC.x + jointF.x) / 2 - 15, (jointC.y + jointF.y) / 2 - 10);
    });

    // fit the render viewport to the scene
    function updateViewport() {
        Render.setSize(render, window.innerWidth, window.innerHeight);
        Render.lookAt(render, {
            min: { x: 0, y: 0 },
            max: { x: window.innerWidth, y: window.innerHeight }
        });
    }

    window.addEventListener('resize', updateViewport);
    updateViewport();

    // context for MatterTools.Demo
    return {
        engine: engine,
        runner: runner,
        render: render,
        canvas: render.canvas,
        stop: function() {
            window.removeEventListener('resize', updateViewport);
            Matter.Render.stop(render);
            Matter.Runner.stop(runner);
        }
    };
};

Example.strandbeast_leg.title = 'Strandbeast Leg Linkage';
Example.strandbeast_leg.for = '>=0.14.2';

if (typeof module !== 'undefined') {
    module.exports = Example.strandbeast_leg;
}
