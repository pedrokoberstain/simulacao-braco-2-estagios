"""Gera o layout do AdvantageScope:  python tools/gerar_layout.py"""
import json
import os
import uuid

R = "/RealOutputs/Arm/"


def fonte(tipo, chave, tipo_log, **opcoes):
    return {"type": tipo, "logKey": R + chave, "logType": tipo_log,
            "visible": True, "options": opcoes}


def aba(tipo, titulo, controle, altura):
    return {"type": tipo, "title": titulo, "controller": controle,
            "controllerUUID": uuid.uuid4().hex, "renderer": None, "controlsHeight": altura}


pose = fonte("robot", "", "Pose2d", model="Braco 2 estagios")
pose["logKey"] = "/RealOutputs/Robo/Pose"
bolas = fonte("gamePiece", "", "Pose3d[]", variant="Algae")
bolas["logKey"] = "/RealOutputs/Jogo/Algas"
campo3d = {
    "sources": [pose, fonte("component", "PosesComponentes", "Pose3d[]"), bolas],
    "game": "FRC:2025 Field (AndyMark)",
}
cores = {"OmbroGraus": "#2b66a2", "EixoGraus": "#e5b31b",
         "PontaRelGraus": "#af2437", "PontaAbsolutaGraus": "#80588e"}
grafico = {
    "leftSources": [fonte("smooth", k, "Number", color=c, size="normal")
                    for k, c in cores.items()],
    "rightSources": [], "discreteSources": [],
    "leftLockedRange": None, "rightLockedRange": None,
    "leftUnitConversion": {"autoTarget": None, "preset": None},
    "rightUnitConversion": {"autoTarget": None, "preset": None},
    "leftFilter": 0, "rightFilter": 0,
}
documentacao = {"type": 0, "title": "", "controller": None,
                "controllerUUID": uuid.uuid4().hex, "renderer": "", "controlsHeight": 0}

layout = {
    "hubs": [{
        "x": 0, "y": 0, "width": 1500, "height": 820,
        "state": {
            "sidebar": {"width": 264, "expanded": [
                "/AdvantageKit", "/RealOutputs", "/RealOutputs/Arm"]},
            "tabs": {"selected": 1, "tabs": [
                documentacao,
                aba(3, "Braco 3D", campo3d, 200),
                aba(10, "Braco 2D",
                    [fonte("mechanism", "Mechanism2d", "Mechanism2d")], 200),
                aba(1, "Angulos", grafico, 200),
            ]},
        },
    }],
    "satellites": [],
    "version": "26.0.0",
}

destino = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                       "advantagescope", "layout.json")
with open(destino, "w", encoding="utf-8", newline="\n") as f:
    json.dump(layout, f, indent=2)
    f.write("\n")
print(os.path.abspath(destino))
