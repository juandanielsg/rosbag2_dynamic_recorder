// Runs the browser UI's script against sample recorder state, with a DOM stub.
//
//     node page_harness.js <script.js> <state.json>
//
// Exists because nothing else executes the page's JavaScript: /api/state answers whether or not
// the page parses, so a parse error would leave the UI frozen behind a green test suite.
//
// This is not a browser and does not pretend to be one. It answers three questions: does the render
// path throw, which is the failure that leaves the page frozen; does text from the ROS graph reach
// markup escaped; and does a render with nothing changed leave the page's elements alone, which is
// what keeps keyboard focus and tooltips alive across polls. Layout and styling are out of scope.

const fs = require("fs");

const [scriptPath, statePath] = process.argv.slice(2);
if (!scriptPath || !statePath) {
  console.error("usage: node page_harness.js <script.js> <state.json>");
  process.exit(2);
}

// --- the smallest DOM the page can run against -------------------------------
const nodes = new Map();
// Markup writes per element and elements created, so a render can be held to touching nothing.
const writes = new Map();
let created = 0;
function element(id) {
  if (nodes.has(id)) return nodes.get(id);
  let html = "";
  const node = {
    id, textContent: "", className: "", title: "", value: "",
    hidden: false, disabled: false, checked: false,
    dataset: {}, style: {}, children: [], listeners: {},
    get innerHTML() { return html; },
    set innerHTML(value) { html = value; writes.set(id, (writes.get(id) || 0) + 1); },
    classList: { toggle() {}, add() {}, remove() {}, contains: () => false },
    appendChild(child) { this.children.push(child); return child; },
    append(...children) { this.children.push(...children); },
    removeChild() {}, remove() {},
    querySelectorAll: () => [],
    querySelector: () => null,
    addEventListener(type, fn) { (this.listeners[type] = this.listeners[type] || []).push(fn); },
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
    createElement: () => { created++; return element("anonymous:" + nodes.size); },
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
// Stopped: the timeline is the last bag's, frozen, and the controls give way to a start.
check("stopped state", () => sandbox.__render(Object.assign({}, state,
  { recording: false, subscribed_topics: [], schedules: [] })));
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

// --- a poll with nothing new must not rebuild anything ----------------------------------------
// The page renders every second. Replacing identical markup dropped keyboard focus, a selection and
// the tooltip under the pointer each time, so a second render of the same state may write nothing.
console.log("unchanged renders:");
check("an unchanged state rebuilds nothing", () => {
  sandbox.__setFilter("text", "");
  sandbox.__render(state);
  writes.clear();
  created = 0;
  sandbox.__render(state);
  const rewritten = [...writes.keys()];
  if (rewritten.length) throw new Error("rewrote the markup of " + rewritten.join(", "));
  if (created) throw new Error(`created ${created} element(s)`);
});

// --- text from the graph is escaped -------------------------------------------------------------
// ~/record takes any uri a ROS client sends, and events and profiles carry names from elsewhere.
// None of it may reach the page as markup.
console.log("escaping:");
const evil = '<img src=x onerror="alert(1)">';
const hostile = JSON.parse(JSON.stringify(state));
const last = hostile.history.length ? hostile.history[hostile.history.length - 1].stamp : 0;
hostile.uri = evil;
hostile.storage_id = evil;
hostile.subscribed_topics = hostile.subscribed_topics.concat([evil]);
hostile.profiles = [{ name: '"><b>profile</b>', topics: [evil] }];
hostile.history = hostile.history.concat([
  { kind: "subscription", topic: evil, action: "subscribed", reason: evil, stamp: last + 1 },
  { kind: "pause", topic: "", action: "paused", reason: evil, stamp: last + 2 },
  { kind: "low_disk", topic: "", action: "stopped", reason: evil, stamp: last + 3 },
  { kind: "split", topic: "", action: "split", reason: evil, stamp: last + 4,
    closed_file: evil, opened_file: "/tmp/" + evil },
]);
hostile.schedules = [{ action: evil, at: hostile.recorder_now + 5, mode: "publish", topic: evil }];
hostile.topic_losses = [{ topic: evil, missed: 1, lost_in_transport: 0, lost_in_recorder: 0 }];
check("text from the graph is escaped", () => {
  sandbox.__render(hostile);
  for (const [id, node] of nodes) {
    if (/<img|<b>|onerror="/.test(node.innerHTML)) {
      throw new Error(`${id} holds raw markup: ${node.innerHTML.slice(0, 300)}`);
    }
  }
});

// --- ticking a box survives the next poll -------------------------------------------------------
console.log("pending edits:");
check("a ticked box survives the next poll", () => {
  sandbox.__render(state);
  const topics = element("topics");
  const handlers = topics.listeners.change || [];
  if (!handlers.length) throw new Error("nothing listens for changes on the topic list");
  for (const fn of handlers) fn({ target: { checked: true, dataset: { topic: "/spike/b" } } });
  writes.clear();
  sandbox.__render(state);
  if (writes.get("topics")) throw new Error("the topic list was rebuilt after a box was ticked");
  if (!/add 1/.test(element("pending").textContent)) {
    throw new Error(`the tick was lost: pending reads "${element("pending").textContent}"`);
  }
});

// Give any pending promise from the load-time poll() a chance to reject before judging.
setTimeout(() => {
  for (const [label, err] of failures) console.error(`\n${label}:\n  ${err && err.stack || err}`);
  console.log(failures.length ? `\n${failures.length} failure(s)` : "\nevery render path ran");
  process.exit(failures.length ? 1 : 0);
}, 50);
