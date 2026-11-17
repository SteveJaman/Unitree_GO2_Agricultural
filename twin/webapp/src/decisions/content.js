// Words and numbers for the Decisions tab: edit here, not in the layout.
// Sources: docs/simulation/traversability_sim.md (phase-4 trials, n = 10 per mode),
// docs/simulation/results/phase4/, docs/hardware/step_over_crawl.md (real dog, 2026-11-16).

/** Height bands. Real dog: 3 cm (its measured step clears 4.1-6.5 cm); simulation: 12 cm. */
export const BANDS = {
  stepOver: { key: 'STEP_OVER', name: 'Step over', tone: 'good', plain: 'low and solid: walk over it' },
  careful: { key: 'CAUTION', name: 'Careful', tone: 'warn', plain: 'high penalty: go around when there is room, slow down near it' },
  avoid: { key: 'AVOID', name: 'Must avoid', tone: 'bad', plain: 'blocked: the route never crosses it' },
  unknown: { key: 'UNKNOWN', name: 'Not decided yet', tone: 'muted', plain: 'no measurement or answer yet' },
};
export const LABEL_BAND = { STEP_OVER: BANDS.stepOver, CAUTION: BANDS.careful, AVOID: BANDS.avoid, UNKNOWN: BANDS.unknown };
export const HEIGHT_ERROR_CM = 1.2;      // median |measured - true| in the simulation (181 boxes)

export const HOW = [
  {
    id: 'problem',
    title: 'The problem, in plain words',
    body: [
      'A walking robot carries out a routine round in a building, for example a facility or greenhouse inspection that takes photos and readings at set points. Floors have small clutter: cables, mats, books, a shoe.',
      'Standard robot navigation treats every object as a wall. The robot walks long ways around things it could simply step over, arrives late, wastes battery, or gets stuck and calls a person to take over. It also cannot tell a cable from a laptop of the same height: one is fine to step on, the other is not.',
      'This project lets the robot decide, object by object, whether to step over, go carefully, or keep away.',
    ],
  },
  {
    id: 'who',
    title: 'Who it is for',
    body: [
      'Facility inspection and maintenance staff who send a legged robot to collect images or sensor readings at set indoor locations. They are busy elsewhere: every unnecessary detour delays the round, and every stop that needs a person to take control interrupts their work.',
      'Legged robots suit buildings designed for people (steps, thresholds, low clutter) where wheels struggle; this is only useful if the robot uses that ability safely.',
      'Not yet evaluated: operator workload and trust. Those need a user study; the work here is the technical side.',
    ],
  },
  {
    id: 'terms',
    title: 'Three words used here',
    terms: [
      ['Traversability', 'whether the robot can move across something: walk over it, go carefully, or not at all.'],
      ['Costmap', 'a map that gives every patch of floor a penalty. The planner picks the route with the lowest total penalty; a "blocked" patch is never crossed.'],
      ['Semantic', 'what the object is (a cable, a laptop, a plant), as opposed to only how tall it is.'],
    ],
  },
  {
    id: 'example',
    title: 'One object, step by step',
    steps: [
      ['The camera sees it', 'YOLO (an object detector) draws a box around a laptop on the floor, 1.1 m ahead.'],
      ['The LiDAR measures it', 'The laser scanner’s points inside that box give a measured height of 6 cm. Measured, not exact: the median error was 1.2 cm.'],
      ['Height alone says', 'On the real dog, 6 cm is above the 3 cm it can safely step over: Careful (high penalty).'],
      ['Meaning is added', 'Laptop is on the operator’s protected list, so it becomes Must avoid, whatever its height. For objects not on the list, a vision-language model (VLM) is asked what the object is; its answer can make the decision more careful, never less.'],
      ['The robot decides', 'The costmap marks the laptop and a 10 cm margin as blocked. The planner routes around it.'],
      ['What happens', 'The dog walks around the laptop. A cable of the same height would only be marked Careful, and a flat 2 cm cable is walked over.'],
    ],
  },
  {
    id: 'protected',
    title: 'Who decides what is protected',
    body: [
      'The operator. The protected list (a file in the project, and editable live in the Live view of this tab) names the object types the robot must never touch: potted plant, vase, laptop, phone, TV, person, cat, dog, wine glass. A detected object of one of those types is Must avoid when the LiDAR measured it, and Careful when it did not.',
      'The VLM’s own judgement is a second layer: it is asked about every object not on the list and can make the decision more careful (fragile, liquid, tangling), never less. It cannot override the height: an object measured taller than the robot can step over stays at least Careful.',
      'Limits: the robot knows object types, not why an object matters to its owner. A cardboard box holding something precious looks like any box. Marking a specific object (rather than a type) as protected is future work.',
    ],
  },
  {
    id: 'operator',
    title: 'How a person can see and change a decision',
    body: [
      'See: the Live view lists every object the robot is tracking with its measured height, what the VLM said, the final decision and the reason in one sentence, plus a timeline of decisions that can be downloaded.',
      'Change: the operator can add or remove protected object types while the robot runs, and change speeds and clearances in Settings. Clicking a point on the Map sends the robot there.',
      'If the robot keeps refusing to move: the System tab answers "why isn’t it moving?" check by check (LiDAR, map, costmap, planner, motion). The operator can drive it with the Drive pad, or give a new goal.',
      'Future work: mark a single object as safe or protected by clicking it, and a user study of whether operators understand and trust these explanations.',
    ],
  },
  {
    id: 'status',
    title: 'Simulation and the real robot',
    body: [
      'The step-over results are from simulation (Gazebo, n = 10 trials per mode). In simulation the legs are animated over a body that is moved directly: the feet lifting is real geometry, the balance is not.',
      'On the real Go2, the detection, measurement, labels, protected list and costmap run live. Unitree’s walking controller cannot raise its feet on command, so the step-over uses our own leg control: each leg lifted 10 cm with the dog level within 2.3° on the other three. The full step-over crawl is still being tested on the robot.',
    ],
  },
];

/** Results, worded as the reviewers asked. Simulation unless marked real. */
export const RESULTS = [
  {
    id: 'clearance',
    q: 'Foot clearance over low items',
    headline: '26% → 53% of foot passes cleared the item',
    detail: [
      'Not "feet twice as high": the share of foot passes over items 10 cm or lower whose foot stayed above the item’s top. Baseline 15 of 58 passes, traversability 25 of 47, traversability + semantics 15 of 37 (41%).',
      'A pass that did not clear means the foot’s lowest point was below the item’s top while over it: it stepped on it or pushed into it (median 2.5 cm below the top). In the baseline all 43 were in the normal gait; with traversability most were in the careful gait.',
      'Foot strikes per metre did not change (0.24 vs 0.23, not significant).',
    ],
    chart: [
      { label: 'Baseline', value: 15, of: 58 },
      { label: 'Traversability', value: 25, of: 47 },
      { label: '+ Semantics', value: 15, of: 37 },
    ],
  },
  {
    id: 'stats',
    q: 'Trials and the statistical test',
    headline: '10 trials × 3 goals = 30 goals per mode; baseline reached 27/30',
    detail: [
      'Goals reached: baseline 27/30, traversability 25/30, semantics 21/30 (Fisher exact: no significant difference). Same 10 seeds (spawn and goals) in every mode.',
      'Clearance: two-proportion z-test with continuity correction, p = 0.008. Because one trial can contribute many foot passes, it was re-checked with each trial counted once: Mann–Whitney U on per-trial rates p = 0.042, Wilcoxon signed-rank paired by seed p = 0.0625 (5 of 5 non-tied seeds improved), and a cluster bootstrap 95% CI of +0.03 to +0.61. The effect holds, more weakly.',
    ],
  },
  {
    id: 'collisions',
    q: 'Collisions',
    headline: 'Counted, not prevented: objects touched 21 / 17 / 17',
    detail: [
      'Over the 10 trials of each mode: objects touched 21 (baseline), 17 (traversability), 17 (semantics); trunk hits 2 / 3 / 2. Not significantly different.',
      'What the backstop does: it changes the walking mode. In 0 of 5,893 samples did the gait stay normal while the costmap saw an obstacle ahead; with the backstop off, 217 of 1,822 (12%) did. That is a change of gait, not a prevented collision.',
    ],
  },
  {
    id: 'unsafe',
    q: '"Zero unsafe labels"',
    headline: '0 of 149 object sightings: no tall object was ever published as passable',
    detail: [
      'Unsafe = an object measured by the LiDAR at 22 cm or taller that was published as Step over or Careful. Checked automatically against the simulation’s ground-truth object list (149 object sightings over 10 trials), not by a person.',
      'The raw VLM did call tall objects passable 6 times; the rule that the VLM can only raise a measured height’s penalty caught all of them.',
      'What it does not show: that the model recognised every object that needed protection. Overall label accuracy was 48%; a fragile laptop was published as Step over 3 times in 8. That is why protected types are now an operator list, not left to the model.',
    ],
  },
  {
    id: 'pair',
    q: 'What semantics adds: two objects, same height',
    headline: 'Laptop 6.0 cm vs cable coil 6.1 cm',
    detail: [
      'Height alone cannot tell them apart. In the trials the cable coil was labelled Step over 6 of 6 times; the fragile laptop was labelled Must avoid 4 of 8 times and Step over 3 times. Semantics separated them about half the time with the model alone; with the protected list the laptop is Must avoid every time it is measured.',
      'Clearance with semantics (41%) is lower than with height alone (53%) partly because the robot steps near fewer objects at all; whether each avoidance was appropriate is what this example and the Live view show.',
    ],
    pair: [
      { name: 'Cable coil', height: '6.1 cm', model: 'Step over 6/6', now: 'Step over / Careful by height' },
      { name: 'Laptop', height: '6.0 cm', model: 'Must avoid 4/8, Step over 3/8', now: 'Must avoid (protected)' },
    ],
  },
  {
    id: 'height',
    q: 'Measured height and the cutoffs',
    headline: 'Median error 1.2 cm; 164 of 181 boxes in the right band',
    detail: [
      'Heights are measured from LiDAR points, not exact (mean error 5.4 cm, pulled up by a few large misses).',
      'All 17 wrong-band boxes were objects just above a cutoff measured just below it (a 12.4 cm shoe, a 13 cm cup at the 12 cm cutoff): the less careful direction. Near a cutoff the decision can go either way; the VLM can still raise it, and the Live view flags objects within 1.5 cm of a cutoff.',
    ],
  },
  {
    id: 'real',
    q: 'On the real robot',
    real: true,
    headline: 'Each leg lifted 10 cm on our control, level within 2.3°',
    detail: [
      'Running live on the Go2: detection, LiDAR height, VLM labels, protected list, costmap; the protected plant is blocked (cost 254).',
      'Per-leg control (our low-level controller, Unitree’s balance off): stand, and a 10 cm lift of each foot on three legs, joint error 0.003–0.010 rad. The full step-over crawl tipped twice in testing and is being fixed (holding torque, earlier stop); it has not yet stepped over an object on the real robot.',
    ],
  },
];
