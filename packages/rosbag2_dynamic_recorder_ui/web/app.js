"use strict";
// Checkboxes the user has touched but not yet applied. Kept separate from server state so a
// poll landing mid-edit cannot silently revert what someone just clicked.
let pending = null;
let state = null;
let busy = false;

// The first event this page saw can trail the recording start by the poll interval and the
// event's own delivery; only a gap wider than that means history was genuinely not observed.
const UNOBSERVED_SLACK_S = 0.5;

// How long a toast stays up, how many recent events the feed lists, and how often the page polls.
const TOAST_MS = 4000;
const EVENT_FEED_LIMIT = 15;
const POLL_INTERVAL_MS = 1000;
// Where the timeline axis is labelled, as fractions of the recording so far.
const TIMELINE_TICKS = [0, 0.25, 0.5, 0.75, 1];

const $ = (id) => document.getElementById(id);
const fmtInt = (n) => n.toLocaleString();
// Every string that reaches markup goes through this. Bag paths, event reasons and profile names
// come from whoever last called the recorder, and ~/record takes any uri a ROS client sends.
const esc = (t) => String(t ?? "").replace(/[&<>"']/g,
  (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));

// Replace an element's markup only when it changed. The page renders on every poll, and
// rewriting identical markup threw away keyboard focus, a text selection and the tooltip under
// the pointer once a second. Every markup write goes through here, so the cache stays true.
function setHTML(el, html) {
  if (el._html === html) return;
  el._html = html;
  el.innerHTML = html;
}

// mm:ss, so the axis ticks line up as a column of equal-width labels.
function clock(seconds) {
  const whole = Math.max(0, Math.round(seconds));
  return `${Math.floor(whole / 60)}:${String(whole % 60).padStart(2, "0")}`;
}

function fmtDuration(s) {
  s = Math.floor(s || 0);
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60);
  return h ? `${h}h ${m}m` : (m ? `${m}m ${s % 60}s` : `${s}s`);
}

function fmtBytes(b) {
  if (!b) return "0 B";
  const u = ["B", "KB", "MB", "GB", "TB"];
  const i = Math.min(Math.floor(Math.log(b) / Math.log(1024)), u.length - 1);
  return `${(b / Math.pow(1024, i)).toFixed(i ? 1 : 0)} ${u[i]}`;
}

function toast(text) {
  let el = $("toast");
  if (!el) { el = document.createElement("div"); el.id = "toast"; document.body.appendChild(el); }
  el.textContent = text;
  clearTimeout(el._t);
  el._t = setTimeout(() => el.remove(), TOAST_MS);
}

// Deliberately conservative. Anything we cannot substantiate is reported as unknown rather than
// shown as a reassuring zero -- messages have been observed missing from a bag while the
// transport reported no loss at all.
function health(s) {
  if (!s.connected) {
    return { cls: "", text: "No recorder found",
             note: `Nothing is publishing ${s.recorder}/status. Is the recorder running?` };
  }
  if (s.stale) {
    return { cls: "bad", text: "Lost contact with the recorder",
             note: `No status update for more than ${s.stale_after_seconds}s.` };
  }
  if (!s.recording) {
    if (s.waiting_for_clock) {
      return { cls: "warn", text: "Waiting for /clock",
               note: "use_sim_time is set. No bag is open until the simulation publishes its " +
                     "clock; nothing is stuck." };
    }
    // A stop the recorder chose is the one a reader most needs explained.
    if (s.stopped_for_low_disk) {
      return { cls: "bad", text: "Stopped — free space ran low",
               note: `The bag filesystem fell below the configured minimum (${fmtBytes(s.free_space_bytes)} ` +
                     `free of ${fmtBytes(s.total_space_bytes)}). Free some space before starting again.` };
    }
    if (s.stopped_for_max_bag_size) {
      return { cls: "warn", text: "Stopped — the bag reached its size limit",
               note: `max_bag_size is ${fmtBytes(s.max_bag_size)}. Starting again opens a new bag.` };
    }
    return { cls: "", text: "Stopped",
             note: "The bag is closed. Starting again opens a new one and restores the same " +
                   "topics." };
  }
  if (s.paused) {
    return { cls: "warn", text: "Paused",
             note: "Subscriptions are still up; messages are arriving but not being written." };
  }
  if (s.write_errors > 0) {
    return { cls: "bad", text: `Recording — ${fmtInt(s.write_errors)} messages could not be written`,
             note: "These reached the recorder but failed to reach the bag; a full disk is the " +
                   "usual cause. The recording is incomplete and is still running." };
  }
  if (s.messages_missed === null) {
    return { cls: "warn", text: "Recording — loss cannot be measured",
             note: "This middleware does not supply publication sequence numbers, so missing " +
                   "messages cannot be detected. Absence of a warning is not evidence of none." };
  }
  if (s.messages_missed > 0) {
    return { cls: "bad", text: `Recording — ${fmtInt(s.messages_missed)} messages missing`,
             note: "Detected from gaps in publisher sequence numbers." };
  }
  return { cls: "ok", text: "Recording", note: "No missing messages detected." };
}

function renderStats(s) {
  const missed = s.messages_missed === null
    ? '<div class="v unknown">unknown</div>'
    : `<div class="v">${fmtInt(s.messages_missed)}</div>`;
  setHTML($("stats"), `
    <div class="stat"><div class="k">Recorded</div><div class="v">${fmtInt(s.messages_written)}</div></div>
    <div class="stat"><div class="k">Missing</div>${missed}</div>
    <div class="stat"><div class="k">${s.recording ? "Running for" : "Recorded for"}</div><div class="v">${fmtDuration(s.elapsed_seconds)}</div></div>
    <div class="stat"><div class="k">Topics</div><div class="v">${s.subscribed_topics.length}</div></div>
    <div class="stat"><div class="k">On disk</div><div class="v">${fmtBytes(s.bag_size_bytes)}</div></div>`);
}

function renderTopics(s) {
  const recorded = new Set(s.subscribed_topics);
  const checked = pending || recorded;
  const list = s.available_topics.slice();
  // A topic can be recorded after its publisher goes away; keep showing it or it would vanish
  // from the UI while still in the bag.
  for (const t of recorded) if (!list.includes(t)) list.push(t);
  list.sort();

  if (!list.length) {
    setHTML($("topics"), '<li class="empty">No topics are being published yet.</li>');
    renderFilterHint(s, 0, 0, matcher());
    return;
  }
  const m = matcher();
  const shown = list.filter((t) => m.test(t));
  renderFilterHint(s, list.length, shown.length, m);

  if (!shown.length) {
    setHTML($("topics"), '<li class="empty">Nothing matches that filter.</li>');
    return;
  }
  const losses = new Map((s.topic_losses || []).map((l) => [l.topic, l]));
  // The markup carries no checked state, so ticking a box does not change it and the next poll
  // leaves the list, and the focus in it, alone. Boxes are set from state afterwards instead.
  setHTML($("topics"), shown.map((t) => {
    const off = !s.available_topics.includes(t);
    return `<li><label>
      <input type="checkbox" data-topic="${esc(t)}">
      <span class="name">${highlighted(t, m.mark)}</span>
      ${off ? '<span class="tag">publisher gone</span>' : ""}
      ${lossTag(losses.get(t))}
    </label></li>`;
  }).join(""));
  for (const box of $("topics").querySelectorAll("input[data-topic]")) {
    box.checked = checked.has(box.dataset.topic);
  }
  renderPending(recorded);
}

// What one topic has lost in this bag, as a tag beside it, or nothing. Missed is left out of the
// count, not counted as zero, where the middleware supplies no sequence numbers.
function lossTag(loss) {
  if (!loss) return "";
  const missed = loss.missed === null ? 0 : loss.missed;
  const total = missed + loss.lost_in_transport + loss.lost_in_recorder;
  if (!total) return "";
  const parts = [
    loss.missed === null ? "missed: unknown" : `missed: ${fmtInt(loss.missed)}`,
    `lost in transport: ${fmtInt(loss.lost_in_transport)}`,
    `dropped by the recorder: ${fmtInt(loss.lost_in_recorder)}`,
  ];
  return `<span class="tag bad" title="${esc(parts.join("\n"))}">${fmtInt(total)} lost</span>`;
}

function renderPending(recorded) {
  if (!pending) { $("btn-apply").disabled = true; $("pending").textContent = ""; return; }
  const add = [...pending].filter((t) => !recorded.has(t));
  const rem = [...recorded].filter((t) => !pending.has(t));
  if (!add.length && !rem.length) {
    pending = null;
    $("btn-apply").disabled = true;
    $("pending").textContent = "";
    return;
  }
  $("btn-apply").disabled = false;
  const bits = [];
  if (add.length) bits.push(`add ${add.length}`);
  if (rem.length) bits.push(`remove ${rem.length}`);
  $("pending").textContent = bits.join(", ") +
    " — topics you did not change keep recording without a gap";
}

function renderProfiles(s) {
  const list = s.profiles || [];
  $("profiles-card").hidden = list.length === 0;
  if (!list.length) return;

  setHTML($("profiles"), list.map((p) => {
    const on = p.name === s.active_profile;
    return `<button class="${on ? "active" : ""}" data-profile="${esc(p.name)}"
      title="${esc(p.topics.join("\n"))}" ${on || !s.recording ? "disabled" : ""}>${esc(p.name)}</button>`;
  }).join(""));
  $("profile-note").textContent = s.active_profile
    ? `Recording the "${s.active_profile}" profile.`
    : "The current selection does not match a profile.";
}

// --- filter over the topic picker -------------------------------------------
// Text mode narrows the list locally. Regex mode does the same, and additionally lets the pattern
// be handed to the recorder as one set_topics call instead of ticking boxes.
let filterMode = "text";

function matcher() {
  const query = $("filter").value.trim();
  const exclude = filterMode === "regex" ? $("exclude").value.trim() : "";
  if (!query) return { query, exclude, test: () => true, ok: true, mark: null };
  if (filterMode === "text") {
    const q = query.toLowerCase();
    return { query, exclude, test: (n) => n.toLowerCase().includes(q), ok: true, mark: q };
  }
  try {
    const re = new RegExp(query);
    // Applied after the pattern, as the recorder applies exclude_regex to the combined set.
    const ex = exclude ? new RegExp(exclude) : null;
    return { query, exclude, test: (n) => re.test(n) && !(ex && ex.test(n)), ok: true, mark: re };
  } catch (err) {
    // Show everything rather than nothing: a half-typed expression should not look like a
    // pattern that legitimately matched no topics, which is the same distinction the recorder
    // makes when it refuses an invalid one.
    return { query, exclude, test: () => true, ok: false, mark: null };
  }
}

function highlighted(name, mark) {
  if (!mark) return esc(name);
  const re = mark instanceof RegExp
    ? mark : new RegExp(mark.replace(/[.*+?^${}()|[\]\\]/g, "\\$&"), "i");
  const hit = name.match(re);
  if (!hit || !hit[0]) return esc(name);
  const at = name.indexOf(hit[0]);
  return esc(name.slice(0, at)) + "<mark>" + esc(hit[0]) + "</mark>" +
         esc(name.slice(at + hit[0].length));
}

function renderFilterHint(s, total, shown, m) {
  const hint = $("filter-hint");
  hint.classList.toggle("bad", !m.ok);
  const canMatch = filterMode === "regex" && m.query && m.ok && shown > 0 && s.recording;
  $("btn-match").disabled = !canMatch;
  $("btn-add-match").disabled = !canMatch;
  $("btn-match").textContent = canMatch ? `Record only these ${shown}` : "Record only matching";
  $("btn-add-match").textContent = canMatch ? `Add these ${shown}` : "Add matching";

  if (!m.ok) { hint.textContent = "Not a valid regular expression."; return; }
  if (!m.query) { hint.textContent = ""; return; }
  const sent = m.exclude
    ? `{regex: "${m.query}", exclude_regex: "${m.exclude}"}` : `{regex: "${m.query}"}`;
  hint.textContent = filterMode === "regex"
    ? `Sent as one call, ${sent}, nothing to enumerate. Add keeps what is already recorded; ` +
      "Record only drops everything else."
    : `Showing ${shown} of ${total}. Switch to Regex to send the pattern to the recorder.`;
}

// --- scheduling ----------------------------------------------------------------
// What the recorder has queued comes from its status, so a schedule set from the CLI shows too.
const SCHEDULE_LABELS = { resume: "Resume", split: "New file", record: "Start recording" };

// A time on the recorder's clock as a person reads it. Under sim time that clock is the
// simulation's, which is not a date.
function clockText(s, at) {
  return s.use_sim_time ? `t=${at.toFixed(1)} s` : new Date(at * 1000).toLocaleTimeString();
}

function renderSchedule(s) {
  $("schedule-card").hidden = false;
  // A recording recorder can roll over or resume later; a stopped one can only be started.
  const actions = s.recording
    ? [["split", "Start a new file"], ["resume", "Resume"]]
    : [["record", "Start recording"]];
  const actionSel = $("sched-action");
  const keepAction = actionSel.value;
  setHTML(actionSel, actions.map(([v, label]) => `<option value="${v}">${label}</option>`).join(""));
  if (actions.some(([v]) => v === keepAction)) actionSel.value = keepAction;

  const topicSel = $("sched-topic");
  const keepTopic = topicSel.value;
  setHTML(topicSel, ['<option value="">any recorded topic</option>'].concat(
    s.subscribed_topics.map((t) => `<option value="${esc(t)}">${esc(t)}</option>`)).join(""));
  if (s.subscribed_topics.includes(keepTopic)) topicSel.value = keepTopic;

  // Record carries no clock choice: a scheduled start always fires on a timer.
  const record = actionSel.value === "record";
  if (record) $("sched-mode").value = "node";
  $("sched-mode").disabled = record;
  const byTimer = $("sched-mode").value === "node";
  topicSel.disabled = record || byTimer;
  $("btn-schedule").disabled = busy || s.waiting_for_clock;

  $("sched-hint").textContent = record
    ? "Opens a new bag at that time, on a timer, at the bag path above if one is given."
    : byTimer
      ? "Fires on a timer, even if no messages arrive. A relative time counts on the " +
        `recorder's clock${s.use_sim_time ? ", which is the simulation's" : ""}.`
      : "Fires on the first message stamped at or after that time, so it needs traffic: if " +
        "the topic goes quiet, it waits.";

  const list = s.schedules || [];
  setHTML($("schedules"), list.length
    ? list.map((p) => {
      const when = p.mode === "node"
        ? `at ${clockText(s, p.at)}, in ${fmtDuration(Math.max(0, p.at - s.recorder_now))}`
        : `when a message on ${p.topic || "any topic"} has ${p.mode} time ${clockText(s, p.at)}`;
      return `<li><span>${esc(SCHEDULE_LABELS[p.action] || p.action)}</span>` +
             `<span class="when">${esc(when)}</span></li>`;
    }).join("") +
      '<li class="empty">Scheduling the same action again replaces it; there is no cancel.</li>'
    : '<li class="empty">Nothing scheduled.</li>');
}

// --- timeline ----------------------------------------------------------------
function renderTimeline(s) {
  const card = $("timeline-card");
  const history = s.history || [];
  const span = s.connected ? s.elapsed_seconds : 0;
  if (!s.connected || !history.length || !(span > 0)) { card.hidden = true; return; }

  const t0 = s.recording_started;
  const t1 = t0 + span;
  // The recorder retains only ten events for a client that connects late, so anything before the
  // oldest one we hold was simply not observed. The chart says so rather than drawing a bar it
  // cannot vouch for -- the same rule as reporting unknown instead of zero.
  const seenFrom = Math.max(t0, Math.min(history[0].stamp, t1));

  const spans = new Map();
  const open = new Map();
  const pauses = [];
  let pausedAt = null;
  const add = (topic, from, to) => {
    if (!(to > from)) return;
    if (!spans.has(topic)) spans.set(topic, []);
    spans.get(topic).push([from, to]);
  };

  for (const e of history) {
    if (e.kind === "pause") {
      if (e.action === "paused") { if (pausedAt === null) pausedAt = e.stamp; }
      else if (pausedAt !== null) { pauses.push([pausedAt, e.stamp]); pausedAt = null; }
      continue;
    }
    // A recorder-initiated stop (low disk, bag size) ends every span at once, which the status
    // already shows; only subscription changes shape the per-topic bars.
    if (e.kind !== "subscription") continue;
    if (e.action === "subscribed") {
      if (!open.has(e.topic)) open.set(e.topic, e.stamp);
    } else {
      add(e.topic, open.has(e.topic) ? open.get(e.topic) : seenFrom, e.stamp);
      open.delete(e.topic);
    }
  }
  if (pausedAt !== null) pauses.push([pausedAt, t1]);
  for (const [topic, from] of open) add(topic, from, t1);
  // Recorded now but never announced within the history we hold: it started before we were
  // watching, so its bar begins where our knowledge does.
  for (const topic of s.subscribed_topics) if (!spans.has(topic)) add(topic, seenFrom, t1);

  const rows = [...spans.keys()].sort();
  if (!rows.length) { card.hidden = true; return; }

  const at = (t) => ((t - t0) / span) * 100;
  const place = (el, from, to) => {
    el.style.left = at(from) + "%";
    el.style.width = (at(to) - at(from)) + "%";
  };
  const unseen = seenFrom > t0 + UNOBSERVED_SLACK_S;
  const div = (cls, parent) => {
    const el = document.createElement("div");
    el.className = cls;
    parent.appendChild(el);
    return el;
  };

  // The right edge is now, so every bar moves on every poll. The elements are rebuilt only when
  // the chart's shape changes -- a topic, a span or a pause appears or ends -- and otherwise kept
  // and moved, so a tooltip under the pointer survives the poll.
  // Rollovers to a new file, drawn as a line across every row: like a pause, a file boundary
  // affects every topic at once.
  const splits = history.filter((e) => e.kind === "split" && e.stamp >= t0 && e.stamp <= t1)
    .map((e) => e.stamp);
  const tl = $("tl");
  const shape = JSON.stringify(
    [rows.map((t) => [t, spans.get(t).length]), pauses.length, splits.length, unseen]);
  if (tl._shape !== shape) {
    tl._shape = shape;
    tl.textContent = "";
    tl._rows = rows.map((topic) => {
      const name = div("tl-name", tl);
      name.textContent = topic;
      name.title = topic;
      const track = div("tl-track", tl);
      const row = {
        unseen: unseen ? div("tl-unseen", track) : null,
        segs: spans.get(topic).map(() => div("tl-seg", track)),
        bands: pauses.map(() => div("tl-band", track)),
        splits: splits.map(() => div("tl-split", track)),
      };
      for (const band of row.bands) band.title = "paused";
      for (const mark of row.splits) mark.title = "new file";
      return row;
    });
    const axis = div("tl-axis", tl);
    tl._ticks = TIMELINE_TICKS.map(() => {
      const tick = document.createElement("span");
      axis.appendChild(tick);
      return tick;
    });
  }
  rows.forEach((topic, i) => {
    const row = tl._rows[i];
    if (row.unseen) place(row.unseen, t0, seenFrom);
    spans.get(topic).forEach(([from, to], k) => place(row.segs[k], from, to));
    pauses.forEach(([from, to], k) => place(row.bands[k], from, to));
    splits.forEach((stamp, k) => { row.splits[k].style.left = at(stamp) + "%"; });
  });
  TIMELINE_TICKS.forEach((f, k) => { tl._ticks[k].textContent = clock(f * span); });

  $("tl-note").textContent = seenFrom > t0 + UNOBSERVED_SLACK_S
    ? "Hatched: before this page connected. The recorder keeps only its ten most recent events " +
      "for a client that joins late, so what happened earlier was not observed."
    : "";
  card.hidden = false;
}

function renderEvents(s) {
  // The newest few, newest first. history is sorted on the recorder's stamp, oldest first.
  const feed = (s.history || []).slice(-EVENT_FEED_LIMIT).reverse();
  if (!feed.length) { setHTML($("events"), '<li class="empty">Nothing yet.</li>'); return; }
  setHTML($("events"), feed.map((e) => {
    // A pause has no topic because it stops every one of them. Saying "all topics" is the
    // honest label, and the distinction is the whole reason the two event kinds are separate.
    if (e.kind === "pause") {
      return `
    <li>
      <span class="ev-${esc(e.action)}">${esc(e.action)}</span>
      <span class="name ev-all">all topics</span>
      <span class="reason">${esc(e.reason)}</span>
    </li>`;
    }
    // A rollover: the file recording continues in, and what caused it. The full paths are in the
    // tooltip; the name alone is what tells one file from the next.
    if (e.kind === "split") {
      const name = (e.opened_file || "").split(/[\\/]/).pop();
      return `
    <li title="${esc(`${e.closed_file} → ${e.opened_file}`)}">
      <span class="ev-split">new file</span>
      <span class="name ev-all">${esc(name)}</span>
      <span class="reason">${esc(e.reason)}</span>
    </li>`;
    }
    // The recorder stopped itself: free space ran low, or the bag reached its size cap.
    if (e.kind !== "subscription") {
      const why = e.kind === "low_disk" ? "low disk" : e.kind === "bag_size_limit" ? "bag size limit" : e.kind;
      return `
    <li>
      <span class="ev-paused">${esc(e.action)}</span>
      <span class="name ev-all">recording (${esc(why)})</span>
      <span class="reason">${esc(e.reason)}</span>
    </li>`;
    }
    return `
    <li>
      <span class="${e.action === "subscribed" ? "ev-sub" : "ev-unsub"}">
        ${e.action === "subscribed" ? "added" : "removed"}</span>
      <span class="name">${esc(e.topic)}</span>
      <span class="reason">${esc(e.reason)}</span>
    </li>`;
  }).join(""));
}

function renderDetail(s) {
  const missed = s.messages_missed === null
    ? '<span class="v unknown">unknown</span><div class="caveat">The middleware does not supply publication sequence numbers here, so this cannot be measured. Not the same as zero.</div>'
    : `${fmtInt(s.messages_missed)}<div class="caveat">Counted from gaps in each publisher's sequence numbers. Independent of whether the transport reported anything.</div>`;
  setHTML($("detail"), `
    <tr><td>Bag</td><td class="name">${esc(s.uri || "—")}</td></tr>
    <tr><td>Storage</td><td>${esc(s.storage_id || "—")}</td></tr>
    <tr><td>Messages recorded</td><td>${fmtInt(s.messages_written)}</td></tr>
    <tr><td>Messages missing</td><td>${missed}</td></tr>
    <tr><td>Failed writes</td><td>${fmtInt(s.write_errors)}
      <div class="caveat">Messages that arrived but could not be written, usually a full disk. Unlike the two figures above these are known-lost with certainty.</div></td></tr>
    <tr><td>Dropped by writer</td><td>${fmtInt(s.messages_lost_in_recorder)}
      <div class="caveat">Arrived, then dropped because the write cache was full or a write failed. The disk is not keeping up: a larger cache, a lighter storage preset, or fewer topics are the remedies.</div></td></tr>
    <tr><td>Loss reported by transport</td><td>${fmtInt(s.messages_lost_in_transport)}
      <div class="caveat">Only what the transport chose to report. Observed reading 0 while messages were genuinely absent, so it is not a guarantee.</div></td></tr>
    <tr><td>Files written</td><td>${s.bag_splits + 1}</td></tr>
    <tr><td>Size on disk</td><td>${fmtBytes(s.bag_size_bytes)}${s.max_bag_size ? ` of ${fmtBytes(s.max_bag_size)} allowed` : ""}
      <div class="caveat">Bytes flushed to disk, not bytes captured. Reads low early on because the writer buffers.</div></td></tr>
    <tr><td>Free space</td><td>${s.total_space_bytes
        ? `${fmtBytes(s.free_space_bytes)} of ${fmtBytes(s.total_space_bytes)}`
        : '<span class="unknown">unknown</span>'}
      <div class="caveat">${s.min_free_space || s.min_free_space_percent
        ? `Recording stops below ${[s.min_free_space && fmtBytes(s.min_free_space), s.min_free_space_percent && `${s.min_free_space_percent}%`].filter(Boolean).join(" or ")}.`
        : "No minimum is configured, so a full disk fails writes rather than stopping the recording."}</div></td></tr>
    <tr><td>Snapshot mode</td><td>${s.snapshot_mode ? "on" : "off"}</td></tr>`);
}

function render(s) {
  state = s;
  $("target").textContent = s.connected
    ? `${s.recorder} — ${s.uri || "no bag"}` : `waiting for ${s.recorder}`;
  const h = health(s);
  $("dot").className = "dot " + h.cls;
  $("health").textContent = h.text;
  $("health-note").textContent = h.note;

  const on = s.connected && s.recording;
  // When stopped, the only useful action is starting again -- so swap the button rather than
  // leaving a row of dead controls and no way forward.
  $("btn-record").hidden = !s.connected || s.recording;
  $("btn-record").disabled = busy;
  $("startbar").hidden = !s.connected || s.recording;
  for (const id of ["btn-pause", "btn-split", "btn-snapshot", "btn-stop"]) {
    $(id).hidden = s.connected && !s.recording;
  }
  $("btn-pause").textContent = s.paused ? "Resume" : "Pause";
  for (const [id, ok] of [["btn-pause", on], ["btn-split", on],
                          ["btn-snapshot", on && s.snapshot_mode], ["btn-stop", on]]) {
    $(id).disabled = !ok || busy;
  }
  $("btn-snapshot").title = s.snapshot_mode
    ? "Flush the buffer to disk" : "Only available when the recorder runs in snapshot mode";

  if (!s.connected) {
    setHTML($("stats"), "");
    setHTML($("topics"), '<li class="empty">Waiting for the recorder…</li>');
    setHTML($("detail"), "");
    $("profiles-card").hidden = true;
    $("schedule-card").hidden = true;
    return;
  }
  $("btn-apply").title = s.recording ? "" : "Start recording before changing topics";
  renderStats(s);
  renderProfiles(s);
  renderSchedule(s);
  renderTopics(s);
  renderTimeline(s);
  renderEvents(s);
  renderDetail(s);
}

async function act(action, payload = {}) {
  busy = true;
  try {
    const res = await fetch("/api/action", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action, ...payload }),
    });
    const out = await res.json();
    if (!out.ok) toast(out.error || "That did not work.");
    return out;
  } catch (err) {
    toast(`Could not reach the recorder: ${err}`);
    return { ok: false };
  } finally {
    busy = false;
    poll();
  }
}

async function poll() {
  try {
    render(await (await fetch("/api/state")).json());
  } catch {
    $("health").textContent = "Cannot reach the UI server";
    $("health-note").textContent = "The page is open but the node serving it is not responding.";
    $("dot").className = "dot bad";
  }
}

$("filter").addEventListener("input", () => { if (state) renderTopics(state); });
// One listener each for the checkboxes and the profile buttons, on their containers, so their
// markup can be replaced without wiring each element again.
$("topics").addEventListener("change", (ev) => {
  const box = ev.target;
  if (!state || !box.dataset || !box.dataset.topic) return;
  const recorded = new Set(state.subscribed_topics);
  if (!pending) pending = new Set(recorded);
  box.checked ? pending.add(box.dataset.topic) : pending.delete(box.dataset.topic);
  renderPending(recorded);
});
$("profiles").addEventListener("click", async (ev) => {
  const btn = ev.target.closest && ev.target.closest("button[data-profile]");
  if (!btn || btn.disabled) return;
  const name = btn.dataset.profile;
  const out = await act("set_profile", { name });
  if (out.ok) { pending = null; toast(`Switched to ${name}.`); }
});
$("exclude").addEventListener("input", () => { if (state) renderTopics(state); });
for (const [id, mode] of [["mode-text", "text"], ["mode-regex", "regex"]]) {
  $(id).onclick = () => {
    filterMode = mode;
    $("mode-text").setAttribute("aria-pressed", String(filterMode === "text"));
    $("mode-regex").setAttribute("aria-pressed", String(filterMode === "regex"));
    // The exclusion and the two send buttons only mean something for a pattern.
    for (const el of ["exclude", "btn-add-match", "btn-match"]) $(el).hidden = mode !== "regex";
    if (state) renderTopics(state);
  };
}
// Both send the pattern and the exclusion as one call. set_topics makes the match the whole
// selection; subscribe_topics adds it to what is already recorded. Topics recorded before and
// after are never torn down either way.
async function sendMatch(action, verb) {
  const m = matcher();
  if (!m.ok || !m.query) return;
  const out = await act(action, { topics: [], regex: m.query, exclude_regex: m.exclude });
  if (out.ok) {
    pending = null;
    toast(`${verb} ${(out.subscribed_topics || []).length} topics.`);
  }
}
$("btn-match").onclick = () => sendMatch("set_topics", "Now recording only these");
$("btn-add-match").onclick = () => sendMatch("subscribe_topics", "Now recording");

$("btn-record").onclick = async () => {
  const out = await act("record", { uri: $("start-uri").value.trim() });
  if (out.ok) { $("start-uri").value = ""; toast("Recording into a new bag."); }
};
for (const id of ["sched-action", "sched-mode"]) {
  $(id).addEventListener("change", () => { if (state && state.connected) renderSchedule(state); });
}
$("btn-schedule").onclick = async () => {
  const action = $("sched-action").value;
  const at = $("sched-at").value.trim();
  if (!at) { toast("Say when: +30s, 14:05, or a date and time."); return; }
  const payload = action === "record"
    ? { at, uri: $("start-uri").value.trim() }
    : { at, mode: $("sched-mode").value,
        topic: $("sched-topic").disabled ? "" : $("sched-topic").value };
  const out = await act(action === "split" ? "split_bagfile" : action, payload);
  if (!out.ok) return;
  $("sched-at").value = "";
  // The recorder runs a node-time schedule at once when its time has already passed.
  const ran = typeof out.scheduled_at === "number" && $("sched-mode").value === "node" &&
    state && out.scheduled_at <= state.recorder_now;
  toast(ran ? "That time had already passed, so it ran now."
            : `Scheduled for ${state ? clockText(state, out.scheduled_at) : out.scheduled_at}.`);
};
$("btn-pause").onclick = () => act(state && state.paused ? "resume" : "pause");
$("btn-split").onclick = () => act("split_bagfile");
$("btn-snapshot").onclick = () => act("snapshot");
$("btn-stop").onclick = () => {
  if (confirm("Stop recording and close the bag? This cannot be undone.")) act("stop");
};
$("btn-apply").onclick = async () => {
  if (!pending) return;
  const topics = [...pending].sort();
  const out = await act("set_topics", { topics });
  if (out.ok) { pending = null; toast(`Now recording ${topics.length} topics.`); }
};

poll();
setInterval(poll, POLL_INTERVAL_MS);
