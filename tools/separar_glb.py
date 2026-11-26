"""Separa o GLB do robô completo (export do Onshape) nos modelos do AdvantageScope.

    python tools/separar_glb.py "caminho/Full W- (2023)End effecter.glb"

Entrada: aba "Full W/ (2023)End effecter", exportada como GLB (Z para cima).
Saída (na pasta de modelos do AdvantageScope):
    model.glb    base: torres, motores, chassi, bumpers e rodas de tank
    model_0.glb  1º estágio, em pé, com o ombro na origem
    model_1.glb  2º estágio + garra, em pé, com o cotovelo na origem
    config.json

Os GLBs já saem nas coordenadas do robô (X frente, Y esquerda, Z cima, metros),
por isso o config.json não precisa girar nem deslocar nada.
"""
import json
import math
import os
import struct
import sys

import numpy as np

POL = 0.0254

# Medidas lidas do próprio modelo, em polegadas, nas coordenadas do CAD.
OMBRO = np.array([35.6, 20.1, 0.5])
COTOVELO = np.array([35.6, 22.9, 28.1])
PONTA = np.array([35.6, -2.6, 0.5])  # centro das rodas da garra
PISO_Z = -15.2

PASTA_SAIDA = os.path.join(os.environ.get("APPDATA", "."), "AdvantageScope", "userAssets",
                           "Robot_Braco2Estagios")


def inclinacao(de, ate):
    """Ângulo em graus em relação à vertical, positivo para trás (+Y do CAD)."""
    return math.degrees(math.atan2(ate[1] - de[1], ate[2] - de[2]))


ANGULO_1 = inclinacao(OMBRO, COTOVELO)
ANGULO_2 = inclinacao(COTOVELO, PONTA)


def ler(caminho):
    with open(caminho, "rb") as f:
        f.read(12)
        n, _ = struct.unpack("<I4s", f.read(8))
        g = json.loads(f.read(n))
        n, _ = struct.unpack("<I4s", f.read(8))
        return g, f.read(n)


def matriz_local(no):
    if "matrix" in no:
        return np.array(no["matrix"], float).reshape(4, 4).T
    x, y, z, w = no.get("rotation", [0, 0, 0, 1])
    m = np.eye(4)
    m[:3, :3] = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]) * np.array(no.get("scale", [1, 1, 1]))
    m[:3, 3] = no.get("translation", [0, 0, 0])
    return m


def caixa(g, i, pai, pontos):
    no = g["nodes"][i]
    m = pai @ matriz_local(no)
    if "mesh" in no:
        for p in g["meshes"][no["mesh"]]["primitives"]:
            a = g["accessors"][p["attributes"]["POSITION"]]
            for c in range(8):
                canto = [a["max"][k] if c >> k & 1 else a["min"][k] for k in range(3)]
                pontos.append((m @ np.array(canto + [1]))[:3])
    for filho in no.get("children", []):
        caixa(g, filho, m, pontos)
    return pontos


def distancia(p, a, b):
    """Distância no plano Y-Z do ponto p ao segmento a-b."""
    p, a, b = p[1:], a[1:], b[1:]
    t = np.clip(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0, 1)
    return np.linalg.norm(p - a - t * (b - a))


def grupo_do_braco(nome, c, tam):
    """Decide em qual modelo vai uma peça da montagem do braço."""
    perto_ombro = np.linalg.norm(c[1:] - OMBRO[1:]) < 3.5
    perto_cotovelo = np.linalg.norm(c[1:] - COTOVELO[1:]) < 3.5
    entre_tubos = 30.2 < c[0] < 41.0
    corrente = nome == ""

    if "Kraken" in nome or "Gear" in nome:
        return "base"
    if "0.25 in" in nome:
        return "base"  # chapas das torres
    if corrente:
        # As correntes de cima correm ao longo do braço, em x = 32,4 e 38,9.
        em_cima = any(abs(c[0] - x) < 0.5 for x in (32.4, 38.9)) and c[2] > -2.5
        return "estagio1" if em_cima else "base"
    if perto_cotovelo:
        # Rolamentos ficam nos tubos do 1º estágio; o resto gira com o 2º.
        return "estagio1" if "Bearing" in nome else "estagio2"
    if perto_ombro:
        if "Bearing" in nome:
            return "estagio1" if entre_tubos else "base"
        if "48t" in nome:
            return "estagio1"
        if nome == "Part 1" and tam[0] < 1.2 and tam[1] > 2.0:
            return "estagio1"  # espaçadores da coroa do ombro
        return "base"          # eixo, coroas de 16T e travas
    if c[2] < -2.5:
        return "base"          # torres e eixo de baixo
    ponta = COTOVELO + 1.25 * (PONTA - COTOVELO)
    return "estagio1" if distancia(c, OMBRO, COTOVELO) < distancia(c, COTOVELO, ponta) \
        else "estagio2"


def classificar(g):
    raiz = g["scenes"][g.get("scene", 0)]["nodes"][0]
    m_raiz = matriz_local(g["nodes"][raiz])
    grupos = {"base": [], "estagio1": [], "estagio2": []}
    for sub in g["nodes"][raiz]["children"]:
        nome_sub = g["nodes"][sub].get("name", "")
        m_sub = m_raiz @ matriz_local(g["nodes"][sub])
        for peca in g["nodes"][sub].get("children", []):
            nome = (g["nodes"][peca].get("name") or "").replace("occurrence of ", "").strip()
            pontos = caixa(g, peca, m_sub, [])
            if not pontos:
                continue
            pontos = np.array(pontos) / POL
            centro = (pontos.min(0) + pontos.max(0)) / 2
            tam = pontos.max(0) - pontos.min(0)
            if "arm assembly" in nome_sub:
                grupo = grupo_do_braco(nome, centro, tam)
            elif "swerve drive" in nome_sub:
                # O robô é tank: saem os módulos swerve e as tampas deles.
                if any(t in nome for t in ("Swerve Module", "module cover", "Pod Cover")):
                    continue
                grupo = "base"
            else:
                grupo = "estagio2"  # garra
            grupos[grupo].append((peca, m_sub))
    return grupos


class Escritor:
    """Monta um GLB novo só com o que os nós escolhidos usam."""

    def __init__(self, g, dados):
        self.g, self.dados = g, dados
        self.saida = {"asset": {"version": "2.0"}, "scene": 0, "scenes": [{"nodes": [0]}],
                      "nodes": [{"children": []}], "meshes": [], "accessors": [],
                      "bufferViews": [], "materials": []}
        self.bin = bytearray()
        self.cache = {}

    def _acrescentar(self, lista, item):
        self.saida[lista].append(item)
        return len(self.saida[lista]) - 1

    def _reusar(self, chave, criar):
        if chave not in self.cache:
            self.cache[chave] = criar()
        return self.cache[chave]

    def _bytes(self, conteudo, extra):
        self.bin.extend(b"\0" * (-len(self.bin) % 4))
        vista = dict(extra, buffer=0, byteOffset=len(self.bin), byteLength=len(conteudo))
        self.bin.extend(conteudo)
        return self._acrescentar("bufferViews", vista)

    def vista(self, i):
        def criar():
            v = self.g["bufferViews"][i]
            ini = v.get("byteOffset", 0)
            extra = {k: v[k] for k in ("byteStride", "target") if k in v}
            return self._bytes(self.dados[ini:ini + v["byteLength"]], extra)
        return self._reusar(("v", i), criar)

    def acessor(self, i):
        def criar():
            a = dict(self.g["accessors"][i])
            a["bufferView"] = self.vista(a["bufferView"])
            return self._acrescentar("accessors", a)
        return self._reusar(("a", i), criar)

    def material(self, i):
        return self._reusar(("m", i),
                            lambda: self._acrescentar("materials", self.g["materials"][i]))

    def malha(self, i):
        def criar():
            primitivas = []
            for p in self.g["meshes"][i]["primitives"]:
                nova = dict(p)
                nova.pop("extensions", None)
                nova["attributes"] = {k: self.acessor(v) for k, v in p["attributes"].items()}
                if "indices" in p:
                    nova["indices"] = self.acessor(p["indices"])
                if "material" in p:
                    nova["material"] = self.material(p["material"])
                primitivas.append(nova)
            return self._acrescentar("meshes", {"primitives": primitivas})
        return self._reusar(("h", i), criar)

    def no(self, i, matriz=None):
        origem = self.g["nodes"][i]
        m = matriz_local(origem) if matriz is None else matriz
        novo = {"matrix": [float(v) for v in m.T.flatten()]}
        if origem.get("name"):
            novo["name"] = origem["name"]
        if "mesh" in origem:
            novo["mesh"] = self.malha(origem["mesh"])
        if origem.get("children"):
            novo["children"] = [self.no(c) for c in origem["children"]]
        return self._acrescentar("nodes", novo)

    def peca(self, i, pai):
        self.saida["nodes"][0]["children"].append(
            self.no(i, pai @ matriz_local(self.g["nodes"][i])))

    def cilindro(self, centro, raio, largura, cor, nome, lados=32):
        """Cilindro com eixo em X do CAD (roda), medidas em metros."""
        pos, nor, idx = [], [], []

        def vertice(p, n):
            pos.append(p)
            nor.append(n)
            return len(pos) - 1

        for lado in range(lados):
            a0, a1 = (2 * math.pi * k / lados for k in (lado, lado + 1))
            am = (a0 + a1) / 2
            n = (0, math.cos(am), math.sin(am))
            q = [vertice((centro[0] + dx, centro[1] + raio * math.cos(a),
                          centro[2] + raio * math.sin(a)), n)
                 for a, dx in ((a0, -largura / 2), (a1, -largura / 2),
                               (a1, largura / 2), (a0, largura / 2))]
            idx += [q[0], q[1], q[2], q[0], q[2], q[3]]
            for s in (-1, 1):
                x = centro[0] + s * largura / 2
                c = vertice((x, centro[1], centro[2]), (s, 0, 0))
                b0, b1 = (vertice((x, centro[1] + raio * math.cos(a),
                                   centro[2] + raio * math.sin(a)), (s, 0, 0))
                          for a in (a0, a1))
                idx += [c, b0, b1] if s > 0 else [c, b1, b0]
        p = np.array(pos, np.float32)
        acessores = []
        for dados, tipo, comp, alvo in ((p, "VEC3", 5126, 34962),
                                        (np.array(nor, np.float32), "VEC3", 5126, 34962),
                                        (np.array(idx, np.uint32), "SCALAR", 5125, 34963)):
            a = {"bufferView": self._bytes(dados.tobytes(), {"target": alvo}),
                 "componentType": comp, "count": len(dados), "type": tipo}
            if dados is p:
                a["min"], a["max"] = p.min(0).tolist(), p.max(0).tolist()
            acessores.append(self._acrescentar("accessors", a))
        mat = self._acrescentar("materials", {"pbrMetallicRoughness": {
            "baseColorFactor": list(cor) + [1.0], "metallicFactor": 0.1,
            "roughnessFactor": 0.8}})
        malha = self._acrescentar("meshes", {"primitives": [{
            "attributes": {"POSITION": acessores[0], "NORMAL": acessores[1]},
            "indices": acessores[2], "material": mat}]})
        self.saida["nodes"][0]["children"].append(
            self._acrescentar("nodes", {"name": nome, "mesh": malha}))

    def gravar(self, caminho, matriz_raiz):
        self.saida["nodes"][0]["matrix"] = [float(v) for v in matriz_raiz.T.flatten()]
        self.bin.extend(b"\0" * (-len(self.bin) % 4))
        self.saida["buffers"] = [{"byteLength": len(self.bin)}]
        texto = json.dumps(self.saida, separators=(",", ":")).encode()
        texto += b" " * (-len(texto) % 4)
        with open(caminho, "wb") as f:
            f.write(struct.pack("<4sII", b"glTF", 2, 28 + len(texto) + len(self.bin)))
            f.write(struct.pack("<I4s", len(texto), b"JSON") + texto)
            f.write(struct.pack("<I4s", len(self.bin), b"BIN\0") + bytes(self.bin))


def translacao(v):
    m = np.eye(4)
    m[:3, 3] = v
    return m


def giro(eixo, graus):
    a = math.radians(graus)
    c, s = math.cos(a), math.sin(a)
    m = np.eye(4)
    if eixo == "z":
        m[:2, :2] = [[c, -s], [s, c]]
    else:  # y
        m[0, 0], m[0, 2], m[2, 0], m[2, 2] = c, s, -s, c
    return m


def main(entrada):
    g, dados = ler(entrada)
    grupos = classificar(g)

    # CAD -> robô: a frente do robô é o -Y do CAD (lado em que a garra coleta).
    cad_para_robo = giro("z", 90)
    centro_no_piso = np.array([OMBRO[0], OMBRO[1], PISO_Z]) * POL
    matrizes = {
        "base": ("model.glb", cad_para_robo @ translacao(-centro_no_piso)),
        # Pivô na origem e estágio girado até ficar em pé.
        "estagio1": ("model_0.glb",
                     giro("y", ANGULO_1) @ cad_para_robo @ translacao(-OMBRO * POL)),
        "estagio2": ("model_1.glb",
                     giro("y", ANGULO_2) @ cad_para_robo @ translacao(-COTOVELO * POL)),
    }

    os.makedirs(PASTA_SAIDA, exist_ok=True)
    for grupo, (arquivo, matriz) in matrizes.items():
        escritor = Escritor(g, dados)
        for peca, pai in grupos[grupo]:
            escritor.peca(peca, pai)
        if grupo == "base":
            # 6 rodas de tank de 6" (representação; o chassi real é o KitBot).
            for lado in (-1, 1):
                for pos in (-11.0, 0.0, 11.0):
                    centro = np.array([OMBRO[0] + lado * 13.0, OMBRO[1] + pos, PISO_Z + 3.0])
                    escritor.cilindro(centro * POL, 3.0 * POL, 1.6 * POL, (0.08, 0.08, 0.08),
                                      "roda tank")
        caminho = os.path.join(PASTA_SAIDA, arquivo)
        escritor.gravar(caminho, matriz)
        print(f"{arquivo}: {len(grupos[grupo])} peças, {os.path.getsize(caminho) / 1e6:.1f} MB")

    zero = {"zeroedRotations": [], "zeroedPosition": [0, 0, 0]}
    config = {"name": "Braco 2 estagios", "rotations": [], "position": [0, 0, 0],
              "cameras": [], "components": [zero, zero]}
    with open(os.path.join(PASTA_SAIDA, "config.json"), "w", encoding="utf-8") as f:
        json.dump(config, f, indent=2)
    print("pasta:", PASTA_SAIDA)

    def comprimento(a, b):
        return np.linalg.norm(b[1:] - a[1:])

    print(f"1º estágio {comprimento(OMBRO, COTOVELO):.2f} pol, "
          f"2º estágio {comprimento(COTOVELO, PONTA):.2f} pol, "
          f"ombro a {OMBRO[2] - PISO_Z:.1f} pol do piso")
    print(f"pose do CAD (guardado): ombro {ANGULO_1:.1f}°, "
          f"ponta relativa {ANGULO_2 - ANGULO_1:.1f}°")


if __name__ == "__main__":
    main(sys.argv[1])
