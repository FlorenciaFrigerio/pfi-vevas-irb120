"""Regresiones C01–C04. El único servidor empleado es el IRC5 simulado en loopback."""
import json
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

import test_rws as rws
import test_revision as revision
from vevas.backend_rws import BackendRWS
from vevas.consola import CAMPOS_GO, validar_go
from vevas.rws_cliente import CancelacionOrden, Cliente, Falla, subdividir


class ControlRWS(unittest.TestCase):
    def test_go_exige_textos_y_checklist_booleano(self):
        for campo in CAMPOS_GO:
            for valor in (True, 123, ["operador"]):
                go = revision.go_valido()
                go[campo] = valor
                self.assertFalse(validar_go(go)[0])

    def test_entorno_del_go_exige_geometria_finita(self):
        entorno = revision.cfg()["entorno"]
        go = {**revision.go_valido(), "entorno": entorno}
        self.assertTrue(validar_go(go)[0])
        for datos in ({"mesa_z_m": float("nan")}, {"sin_objetos_externos_confirmado": "false"},
                      {"cajas": [{"min_mm": [1, 0, 0], "max_mm": [0, 1, 1]}]},
                      {"cajas": [{"min_mm": [0, 0, 0], "max_mm": [1, 1, float("inf")]}]}):
            self.assertFalse(validar_go({**go, "entorno": {**entorno, **datos}})[0])

    def test_home_real_rechaza_la_llegada_singular_a_cero(self):
        con, b = revision.real_listo()
        self.addCleanup(revision.cerrar, con, b)
        p = con.calcular_home("cero")
        self.assertFalse(p["ejecutable"])
        self.assertEqual(revision.chequeo(p, "singularidad")["estado"], "rechazado")

    def test_mesa_sola_deja_objetos_externos_sin_verificar(self):
        con, b = revision.nueva()
        self.addCleanup(revision.cerrar, con, b)
        ent = con.cfg["entorno"]
        ent["sin_objetos_externos_confirmado"] = False
        ent["cajas"] = [{"origen": "mesa", "min_mm": [800, 800, 0], "max_mm": [900, 900, 100]}]
        p = con.calcular_home("inicial")
        self.assertEqual(revision.chequeo(p, "entorno")["estado"], "no_verificado")
        ent["cajas"][0].update(min_mm=[-1000, -1000, -1000], max_mm=[1000, 1000, 1000])
        p = con.calcular_home("inicial")
        self.assertEqual(revision.chequeo(p, "entorno")["estado"], "rechazado")

    def test_go_en_disco_revocado_bloquea_antes_del_envio(self):
        con, b = revision.real_listo()
        self.addCleanup(revision.cerrar, con, b)
        with tempfile.TemporaryDirectory() as carpeta:
            ruta = Path(carpeta, "go.json")
            ruta.write_text(json.dumps(con.go), encoding="utf-8")
            con.ruta_go = ruta
            self.assertTrue(con.armar(revision.CHECK, revision.FRASE)[0])
            con.calcular_junta(1, 2.0, True)
            ruta.write_text(json.dumps({**con.go, "decision": "NO-GO"}), encoding="utf-8")
            self.assertFalse(con.ejecutar()[0])
            self.assertFalse(b.enviado)
            ruta.unlink()
            self.assertFalse(con.armar(revision.CHECK, revision.FRASE)[0])

    def test_envio_rechaza_id_de_plan_anterior(self):
        con, b = revision.real_listo()
        self.addCleanup(revision.cerrar, con, b)
        self.assertTrue(con.armar(revision.CHECK, revision.FRASE)[0])
        anterior = con.calcular_junta(1, 1.0, True)
        actual = con.calcular_junta(1, 2.0, True)
        self.assertNotEqual(actual["id"], anterior["id"])
        for incorrecto in (anterior["id"], True, str(actual["id"])):
            ok, msg = con.ejecutar(plan_id=incorrecto)
            self.assertFalse(ok, msg)
            self.assertFalse(b.enviado)

    def test_controlador_con_speeddata_fija_bloquea_envio_real(self):
        e = rws.Escenario()
        self.addCleanup(e.cerrar)
        con, b = rws.consola_rws(e, backend_cls=BackendRWS)
        self.addCleanup(rws.cerrar_consola, con, b)
        self.assertTrue(con.armar(revision.CHECK, revision.FRASE)[0])
        p = con.calcular_junta(1, 5.0, True)
        self.assertEqual(revision.chequeo(p, "velocidad_controlador")["estado"], "no_verificado")
        self.assertFalse(p["ejecutable"])
        self.assertFalse(con.ejecutar()[0])
        self.assertFalse(b.ejecutar_trayectoria([], [])[0])
        self.assertFalse(e.mock.ordenes)

    def test_cancelacion_despues_de_target_antes_de_seq(self):
        e = rws.Escenario()
        self.addCleanup(e.cerrar)
        señal = CancelacionOrden()
        leer = e.cli._leer_objetivo

        def al_leer():
            señal.set()
            return leer()

        with patch.object(e.cli, "_leer_objetivo", side_effect=al_leer):
            ok, _, _ = e.cli.enviar_objetivo(rws.HOME, cancelar=señal)
        self.assertFalse(ok)
        self.assertEqual(e.mock.pers["vvPcSeq"], 0)
        self.assertEqual(e.mock.ordenes, [])
        self.assertIsNotNone(e.mock.pers["vvPcTarget"])

    def test_parar_durante_lectura_de_vuelta_no_inicia_el_primer_tramo(self):
        e = rws.Escenario()
        self.addCleanup(e.cerrar)
        con, b = rws.consola_rws(e)
        self.addCleanup(rws.cerrar_consola, con, b)
        self.assertTrue(con.armar(revision.CHECK, revision.FRASE)[0])
        con.calcular_junta(1, 5.0, True)
        entrada, seguir = threading.Event(), threading.Event()
        leer = e.cli._leer_objetivo

        def al_leer():
            entrada.set()
            if not seguir.wait(5):
                raise AssertionError("No se recibió la cancelación.")
            return leer()

        resultado = []
        with patch.object(e.cli, "_leer_objetivo", side_effect=al_leer):
            hilo = threading.Thread(target=lambda: resultado.append(con.ejecutar()))
            hilo.start()
            try:
                self.assertTrue(entrada.wait(5))
                con.parar()
            finally:
                seguir.set()
                hilo.join(8)
        self.assertFalse(resultado[0][0])
        self.assertFalse(con.armado)
        self.assertEqual(e.mock.pers["vvPcSeq"], 0)

    def test_origen_y_pp_se_verifican_antes_de_ordenar(self):
        e = rws.Escenario()
        self.addCleanup(e.cerrar)
        q = list(rws.HOME)
        q[0] += 2
        ok, _, msg = e.cli.enviar_objetivo(q, origen_deg=q)
        self.assertFalse(ok)
        self.assertIn("origen revisado", msg)
        estado = e.cli.leer_estado(con_juntas=False)
        for modulo in ("?", "Module1"):
            estado["pp_modulo"] = modulo
            self.assertTrue(e.cli.problemas_para_enviar(estado))
        self.assertEqual(e.mock.pers["vvPcSeq"], 0)

    def test_origen_caduca_mientras_espera_la_escritura(self):
        e = rws.Escenario()
        self.addCleanup(e.cerrar)
        e.cli.max_edad_juntas_s = 0.15
        espera = threading.Event()
        ok, _, msg = e.cli.enviar_objetivo(rws.HOME, origen_deg=rws.HOME,
                                           al_comprometer=lambda: espera.wait(0.2))
        self.assertFalse(ok)
        self.assertIn("caducó", msg)
        self.assertEqual(e.mock.pers["vvPcSeq"], 0)

    def test_cancelacion_justo_antes_del_transporte(self):
        e = rws.Escenario()
        self.addCleanup(e.cerrar)
        señal = CancelacionOrden()
        ok, _, msg = e.cli.enviar_objetivo(rws.HOME, cancelar=señal, al_comprometer=señal.set)
        self.assertFalse(ok)
        self.assertIn("cancelada", msg)
        self.assertEqual(e.mock.pers["vvPcSeq"], 0)

    def test_otro_hilo_del_mismo_cliente_no_entra_al_candado(self):
        with tempfile.TemporaryDirectory() as carpeta:
            cli = Cliente("localhost", ruta_candado=str(Path(carpeta, "candado")))
            errores = []
            def otro_hilo():
                try:
                    with cli.escritor():
                        pass
                except Falla as ex:
                    errores.append(str(ex))
            with cli.escritor(), cli.escritor():
                t = threading.Thread(target=otro_hilo)
                t.start()
                t.join(1)
                self.assertFalse(t.is_alive())
            self.assertEqual(len(errores), 1)
            with cli.escritor():
                pass

    def test_no_instala_con_lectura_de_vuelta_truncada(self):
        for instalado in (False, True):
            e = rws.Escenario(listo=False)
            try:
                if instalado:
                    e.cli.instalar_modulo(rws.MOD, eco=lambda *_: None)
                with patch.object(e.cli, "_texto_modulo", return_value="MODULE VEVAS_RWS\nENDMODULE"):
                    with self.assertRaises(Falla):
                        e.cli.instalar_modulo(rws.MOD, eco=lambda *_: None)
                self.assertEqual(e.mock.ctrlstate, "motoroff")
                self.assertEqual(e.mock.exec, "stopped")
            finally:
                e.cerrar()

    def test_parar_requiere_ambos_estados_confirmados(self):
        cli = Cliente("localhost")
        with patch.object(cli, "post"), patch("vevas.rws_cliente.time.sleep"):
            for ejec, motores in (("?", "motoroff"), ("stopped", "motoron"), ("running", "motoroff")):
                with patch.object(cli, "leer_estado", return_value={"exec": ejec, "ctrlstate": motores}):
                    self.assertFalse(cli.parar()["ok"])

    def test_las_esquinas_y_el_retorno_no_se_recortan(self):
        a = [0] * 6
        b, c = [4, 4, 0, 0, 0, 0], [8, 0, 0, 0, 0, 0]
        self.assertEqual(subdividir(a, [b, c, a], 10), [b, c, a])
        intermedio = [2, 2, 0, 0, 0, 0]
        self.assertEqual(subdividir(a, [intermedio, b, a], 10), [b, a])


if __name__ == "__main__":
    unittest.main()
