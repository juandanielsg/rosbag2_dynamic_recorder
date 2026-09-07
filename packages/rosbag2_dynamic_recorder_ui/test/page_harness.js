// Runs the browser UI's script against sample recorder state, with a DOM stub.
//
//     node page_harness.js <script.js> <state.json>
//
// Exists because nothing in this project ever executed the page's JavaScript. A parse error in it
// went unnoticed for two days: the unit tests only cover build_state, and /api/state answers
// whether or not the page parses, so every check passed while the page showed nothing but its
// static placeholder text.
//
// This is not a browser and does not pretend to be one. It answers one question -- does the render
// path throw -- which is the failure that leaves the page frozen. Layout, styling and event
// handling are out of scope.

const fs = require("fs");

const [scriptPath, statePath] = process.argv.slice(2);
if (!scriptPath || !statePath) {
  console.error("usage: node page_harness.js <script.js> <state.json>");
  process.exit(2);
}

// --- the smallest DOM the page can run against -------------------------------
const nodes = new Map();
function element(id) {
  if (nodes.has(id)) return nodes.get(id);
  const node = {
    id, textContent: "", innerHTML: "", className: "", title: "", value: "",
    hidden: false, disabled: false, checked: false,
    dataset: {}, style: {}, children: [],
    classList: { toggle() {}, add() {}, remove() {}, contains: () => false },
    appendChild(child) { this.children.push(child); return child; },
    append(...children) { this.children.push(...children); },
    removeChild() {}, remove() {},
    querySelectorAll: () => [],
    querySelector: () => null,
    addEventListener() {},
    setAttribute() {}, getAttribute: () => null,
    focus() {},
    set onclick(_fn) {}, get onclick() { return null; },
  };
  nodes.set(id, node);
  return node;
}

const state = JSON.parse(fs.readFileSync(statePath, "utf8"));
const disconnected = {
  connected: false,
  recorder: state.recorder,
  available_topics: [],
  events: [],
};

// The page's functions are declarations in the script's own scope, so they are exported
// deliberately. If one is renamed this fails loudly, which is the point: the test guards them.
const source = fs.readFileSync(scriptPath, "utf8") + `
;globalThis.__render = render;
globalThis.__setFilter = (mode, query) => {
  filterMode = mode;
  document.getElementById("filter").value = query;
};
`;

const sandbox = {
  document: {
    getElementById: element,
    createElement: () => element("anonymous:" + nodes.size),
    body: element("body"),
  },
  fetch: async () => ({ ok: true, json: async () => state }),
  setInterval: () => 0,
  setTimeout: () => 0,
  clearTimeout: () => {},
  confirm: () => false,
  alert: () => {},
  console,
};
sandbox.globalThis = sandbox;

const failures = [];
process.on("unhandledRejection", (err) => {
  failures.push(["a promise rejected (poll or an action)", err]);
});

try {
  new Function(...Object.keys(sandbox), source)(...Object.values(sandbox));
} catch (err) {
  console.error("the script threw while loading: " + err);
  process.exit(1);
}

function check(label, run) {
  try {
    run();
    console.log("  ok    " + label);
  } catch (err) {
    failures.push([label, err]);
    console.log("  FAIL  " + label);
  }
}

console.log("render paths:");
check("connected state", () => sandbox.__render(state));
check("rendered twice", () => sandbox.__render(state));
check("disconnected state", () => sandbox.__render(disconnected));
check("back to connected", () => sandbox.__render(state));

// The filter has the most branches, and none of them may throw -- least of all the invalid
// pattern, which a user produces simply by typing an opening bracket.
for (const [mode, query] of [
  ["text", ""],
  ["text", "spike"],
  ["text", "matches-nothing"],
  ["regex", "^/spike/"],
  ["regex", "["],
  ["regex", "(unclosed"],
  ["regex", "matches-nothing"],
]) {
  check(`filter ${mode} ${JSON.stringify(query)}`, () => {
    sandbox.__setFilter(mode, query);
    sandbox.__render(state);
  });
}

// Give any pending promise from the load-time poll() a chance to reject before judging.
setTimeout(() => {
  for (const [label, err] of failures) console.error(`\n${label}:\n  ${err && err.stack || err}`);
  console.log(failures.length ? `\n${failures.length} failure(s)` : "\nevery render path ran");
  process.exit(failures.length ? 1 : 0);
}, 50);
