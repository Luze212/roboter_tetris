import fs from "node:fs/promises";
import path from "node:path";
import { pathToFileURL } from "node:url";
import { Presentation, PresentationFile } from "@oai/artifact-tool";

const workspaceDir = process.env.WORKSPACE_DIR;
const skillDir = process.env.SKILL_DIR;
const runtimePython = process.env.RUNTIME_PYTHON;
const buildDir = process.env.BUILD_DIR;
const figureDir = path.join(workspaceDir, "Projektbericht", "abbildungen");
const candidatePath = path.join(buildDir, "systemskizzen-candidate.pptx");
const finalPath = path.join(figureDir, "systemskizzen_layout_v02.pptx");

const { resolvePresentationFont, finalizePresentation } = await import(
  pathToFileURL(path.join(skillDir, "container_tools", "artifact_tool_utils.mjs")).href,
);
const family = resolvePresentationFont();

const C = {
  ink: "#17212B", muted: "#4B5563", blue: "#1D70B8", lightBlue: "#E7EEF4",
  bandEdge: "#305B78", orange: "#E87520", camera: "#D6DCE1", robot: "#DBE8D6",
  crate: "#F5DFBD", crateEdge: "#9B6622", green: "#46734A", white: "#FFFFFF",
};

function text(slide, value, left, top, width, height, size = 20, options = {}) {
  const shape = slide.shapes.add({
    geometry: "textbox", position: { left, top, width, height }, fill: "none",
    line: { fill: "none", width: 0 },
  });
  shape.text = value;
  shape.text.style = {
    typeface: family, fontSize: size, color: options.color ?? C.ink,
    bold: options.bold ?? false, alignment: options.alignment ?? "left", autoFit: "shrinkText",
  };
  return shape;
}

function rect(slide, left, top, width, height, fill, line = { style: "solid", fill: C.ink, width: 2 }, geometry = "rect") {
  return slide.shapes.add({ geometry, position: { left, top, width, height }, fill, line });
}

function line(slide, left, top, width, height, color, dash = "solid", thickness = 2) {
  const normalizedLeft = width < 0 ? left + width : left;
  const normalizedTop = height < 0 ? top + height : top;
  return slide.shapes.add({
    geometry: "line", position: {
      left: normalizedLeft, top: normalizedTop, width: Math.abs(width) || 1, height: Math.abs(height) || 1,
    }, fill: "none",
    line: { style: dash, fill: color, width: thickness },
  });
}

function triangle(slide, left, top, width, height, color, rotation = 0) {
  return slide.shapes.add({
    geometry: "triangle", position: { left, top, width, height, rotation }, fill: color,
    line: { fill: color, width: 0 },
  });
}

function title(slide, heading, subtitle) {
  text(slide, heading, 64, 40, 1120, 46, 32, { bold: true });
  text(slide, subtitle, 64, 92, 1120, 28, 17, { color: C.muted });
}

function planSlide(presentation) {
  const slide = presentation.slides.add();
  slide.background.fill = C.white;
  title(slide, "Draufsicht des Versuchsaufbaus", "Globale Koordinaten im Bezugssystem world");

  // Förderband und greifbarer Arbeitsbereich
  rect(slide, 160, 150, 420, 420, C.lightBlue, { style: "solid", fill: C.bandEdge, width: 3 }, "roundRect");
  text(slide, "Förderband", 290, 170, 150, 30, 21, { bold: true, alignment: "center" });
  rect(slide, 200, 348, 354, 209, "none", { style: "dashed", fill: C.blue, width: 2 });
  line(slide, 370, 250, 0, 235, C.orange, "solid", 5);
  triangle(slide, 358, 480, 24, 26, C.orange, 180);

  // Base-Kamera am Bandanfang
  rect(slide, 330, 122, 80, 22, C.camera, { style: "solid", fill: C.muted, width: 2 }, "roundRect");
  line(slide, 370, 144, 0, 6, C.muted, "solid", 2);
  text(slide, "Base-Kamera", 64, 138, 170, 28, 20, { bold: true });
  text(slide, "mittig am Bandanfang\nca. 0,850 m über Band", 64, 170, 210, 48, 16, { color: C.muted });

  // Kalibrierungssystem conveyor_frame
  line(slide, 370, 150, -70, 0, C.blue, "solid", 2);
  triangle(slide, 286, 143, 16, 14, C.blue, 270);
  line(slide, 370, 150, 0, 70, C.blue, "solid", 2);
  triangle(slide, 363, 207, 14, 16, C.blue, 180);
  text(slide, "X+", 285, 124, 35, 18, 14, { color: C.blue });
  text(slide, "Y+", 385, 208, 35, 18, 14, { color: C.blue });
  text(slide, "conveyor_frame", 215, 225, 155, 20, 14, { color: C.blue });

  // Ablagekiste und roboterbasisbezogenes globales System
  rect(slide, 626, 285, 120, 72, C.crate, { style: "solid", fill: C.crateEdge, width: 2 }, "roundRect");
  text(slide, "Ablagekiste", 638, 309, 96, 23, 16, { alignment: "center" });
  const base = rect(slide, 794, 430, 112, 112, C.robot, { style: "solid", fill: C.green, width: 3 }, "ellipse");
  text(slide, "UR10e", 812, 510, 76, 22, 17, { alignment: "center" });
  line(slide, 850, 486, 78, 0, C.blue, "solid", 2);
  triangle(slide, 924, 479, 14, 14, C.blue, 90);
  line(slide, 850, 486, 0, -68, C.blue, "solid", 2);
  triangle(slide, 843, 414, 14, 14, C.blue, 0);
  text(slide, "X+", 918, 458, 35, 18, 14, { color: C.blue });
  text(slide, "Y+", 865, 400, 35, 18, 14, { color: C.blue });
  text(slide, "world", 940, 500, 70, 18, 14, { color: C.blue });

  // Beschriftungsspalte ohne kreuzende Linien
  text(slide, "Greifbarer Arbeitsbereich", 970, 200, 250, 30, 21, { bold: true, color: C.blue });
  text(slide, "x = -1,200 bis -0,530 m\ny = -0,320 bis 0,430 m", 970, 238, 250, 52, 18, { color: C.blue });
  text(slide, "Ablagepose", 970, 350, 170, 28, 21, { bold: true });
  text(slide, "x = -0,316 m\ny = 0,476 m\nz = 0,420 m", 970, 388, 190, 70, 18);
  text(slide, "Bandkanten: x = -1,275 bis -0,480 m", 160, 610, 390, 24, 17, { color: C.muted });
  text(slide, "Nutzbare gerade Förderstrecke: ca. 1,507 m", 160, 642, 430, 24, 17, { color: C.muted });
  text(slide, "Z+ zeigt von der Bandebene nach oben und ist in der Draufsicht nicht dargestellt.", 64, 682, 900, 20, 15, { color: C.muted });
  slide.speakerNotes.textFrame.setText("Eigene Vermessungen und Projektkonfiguration, Stand 24.09.2026.");
  return slide;
}

function sideSlide(presentation) {
  const slide = presentation.slides.add();
  slide.background.fill = C.white;
  title(slide, "Seitenansicht der relevanten Höhen", "Flanschhöhen im globalen Bezugssystem world");

  // Kamerarahmen und Förderband
  line(slide, 210, 150, 0, 375, C.muted, "solid", 6);
  line(slide, 710, 150, 0, 375, C.muted, "solid", 6);
  line(slide, 210, 150, 500, 0, C.muted, "solid", 6);
  rect(slide, 437, 162, 76, 28, C.camera, { style: "solid", fill: C.muted, width: 2 }, "roundRect");
  text(slide, "Base-Kamera", 560, 168, 190, 30, 21, { bold: true });
  text(slide, "ca. 0,850 m über Band", 560, 202, 220, 24, 18, { color: C.muted });
  rect(slide, 150, 525, 620, 44, C.lightBlue, { style: "solid", fill: C.bandEdge, width: 3 }, "roundRect");
  text(slide, "Förderband", 370, 533, 180, 24, 20, { bold: true, alignment: "center" });

  // Höhenlinien und dauerhaft getrennte Beschriftungen
  const levels = [
    { y: 278, labelY: 260, label: "obere Arbeitsraumgrenze: z = 0,600 m", color: C.blue, dash: "solid" },
    { y: 360, labelY: 330, label: "Transferhöhe: z = 0,490 m", color: "#5D91C0", dash: "dashed" },
    { y: 390, labelY: 402, label: "Folge- und Beobachtungshöhe: z = 0,450 m", color: "#5D91C0", dash: "dashed" },
    { y: 475, labelY: 456, label: "untere Arbeitsraumgrenze: z = 0,309 m", color: C.blue, dash: "solid" },
  ];
  for (const item of levels) {
    line(slide, 230, item.y, 540, 0, item.color, item.dash, item.dash === "solid" ? 2 : 1);
    line(slide, 770, item.y, 22, item.labelY - item.y + 10, item.color, "solid", 1);
    text(slide, item.label, 810, item.labelY, 390, 26, 18, { color: item.color });
  }

  // Höhendifferenz der Kamera und Z-Achse
  line(slide, 100, 525, 0, -315, C.orange, "solid", 3);
  triangle(slide, 93, 205, 14, 16, C.orange, 0);
  text(slide, "ca. 0,850 m", 48, 350, 140, 24, 18, { color: C.orange });
  line(slide, 1130, 525, 0, -230, C.blue, "solid", 3);
  triangle(slide, 1123, 288, 14, 16, C.blue, 0);
  text(slide, "Z+", 1150, 294, 36, 22, 18, { color: C.blue });
  text(slide, "world", 1095, 552, 70, 20, 14, { color: C.blue });
  text(slide, "Die Höhen betreffen den Flansch ur_tool0. Der Greifer ist nicht dargestellt.", 64, 682, 900, 20, 15, { color: C.muted });
  slide.speakerNotes.textFrame.setText("Eigene Vermessungen und Projektkonfiguration, Stand 24.09.2026.");
  return slide;
}

await fs.mkdir(buildDir, { recursive: true });
await fs.mkdir(figureDir, { recursive: true });
const presentation = Presentation.create({ slideSize: { width: 1280, height: 720 } });
const plan = planSlide(presentation);
const side = sideSlide(presentation);
await (await PresentationFile.exportPptx(presentation)).save(candidatePath);

for (const [slide, name] of [[plan, "systemaufbau_draufsicht_ppt.png"], [side, "systemaufbau_seitenansicht_ppt.png"]]) {
  const preview = await presentation.export({ slide, format: "png", scale: 2 });
  await fs.writeFile(path.join(figureDir, name), new Uint8Array(await preview.arrayBuffer()));
}

const result = await finalizePresentation({
  explicitTotalSlideCount: 2,
  workspaceDir,
  candidatePath,
  finalPath,
  pythonExecutable: runtimePython,
  integrityValidatorPath: path.join(skillDir, "container_tools", "inspect_presentation_package_integrity.py"),
  layoutValidatorPath: path.join(skillDir, "container_tools", "inspect_presentation_layout_geometry.py"),
  layoutArgs: ["--expected-slide-size-emu", "12192000,6858000", "--validate-heading-fit"],
  requiredNativeTableOwnerSlides: [],
  fontPolicy: { basis: "design", families: [family] },
  verifyArtifactToolImport: true,
  receiptPath: path.join(buildDir, "systemskizzen_layout_v02.validation.json"),
});
console.log(JSON.stringify({ finalPath, result }, null, 2));
