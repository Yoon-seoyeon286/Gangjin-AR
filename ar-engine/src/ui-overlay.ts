// ui-overlay.ts
// Recreated to fix build errors

declare const ecs: any; // Externally defined or needs import in real env

// Missing helper function stubs
const updateTimer = (time: number) => console.log('Timer:', time);
const createUI = () => console.log('Create UI');
const updateScore = (score: number) => console.log('Score:', score);
const updateCrackCount = (cracks: number) => console.log('Cracks:', cracks);
const debugLog = (msg: string) => console.log('Debug:', msg);
const initialize = () => true;

// Mock component context variables usually available in the function scope or passed in
const dataAttribute = {};
const eid = 0;

// Corrected syntax for state definition
ecs.defineState('active')
    .initial()
    .onEnter(() => {
        debugLog('Entered active state');
        createUI();
    });

export const uiOverlay = ecs.registerComponent({
    name: 'ui-overlay',
    schema: {
        debugMode: { type: 'boolean', default: false },
    },
    data: {
        initialized: { type: 'boolean', default: false },
        currentScore: { type: 'int', default: 0 },
        currentCracks: { type: 'int', default: 0 },
        startTime: { type: 'number', default: 0 },
    },
    init: function () {
        debugLog('Initializing ui-overlay');
        initialize();
    },
    tick: function (time: number, timeDelta: number) {
        if (this.data.initialized) {
            updateTimer(time);
        }
    },
});
