from collections import deque
from sortedcontainers import SortedSet # biblioteca para importar arvores balanceadas
import heapq
import time
import os
import sys
import tracemalloc

class MundoBlocosCorreto:
    def __init__(self, texto_strips):
        linhas = [linha.strip() for linha in texto_strips.strip().split('\n') if linha.strip()]
        
        self.acoes = {}
        self.estado_inicial = []
        self.estado_final = []
        
        # Últimas 2 linhas: estado inicial e final (conforme parser original)
        if len(linhas) >= 2:
            self.estado_inicial = [p.strip() for p in linhas[-2].split(';') if p.strip()]
            self.estado_final = [p.strip() for p in linhas[-1].split(';') if p.strip()]
            linhas_acoes = linhas[:-2]
        else:
            linhas_acoes = linhas
        
        # Processar ações em grupos de 3
        i = 0
        while i < len(linhas_acoes):
            nome_acao = linhas_acoes[i]
            precondicoes = []
            efeitos = []
            if i + 1 < len(linhas_acoes):
                precondicoes = [p.strip() for p in linhas_acoes[i+1].split(';') if p.strip()]
            if i + 2 < len(linhas_acoes):
                efeitos = [e.strip() for e in linhas_acoes[i+2].split(';') if e.strip()]
            
            self.acoes[nome_acao] = {
                'pre': precondicoes,
                'efeitos': efeitos
            }
            i += 3
        
        # Criar mapeamento considerando proposições negadas também
        self._criar_mapeamento()
        
        # Converter estados
        self.estado_inicial_int = self._para_vetor(self.estado_inicial, eh_final=False)
        # Estado final pode conter negações (ex.: ~on_a_b) -> mantemos sinais
        self.estado_final_int = self._para_vetor(self.estado_final, eh_final=True)
        
        # Preparar ações (pré podem ser negativas; efeitos: add/del)
        self.acoes_preparadas = self._preparar_acoes()


    def _criar_mapeamento(self):
        
        todas_proposicoes = set()
        
        for prop in self.estado_inicial:
            if prop:
                if prop.startswith('~'):
                    todas_proposicoes.add(prop[1:].strip())
                else:
                    todas_proposicoes.add(prop.strip())
        

        for prop in self.estado_final:
            if prop:
                if prop.startswith('~'):
                    todas_proposicoes.add(prop[1:].strip())
                else:
                    todas_proposicoes.add(prop.strip())
        
        for nome_acao, dados in self.acoes.items():
            for pre in dados['pre']:
                if pre:
                    if pre.startswith('~'):
                        todas_proposicoes.add(pre[1:].strip())
                    else:
                        todas_proposicoes.add(pre.strip())
            for efeito in dados['efeitos']:
                if efeito:
                    if efeito.startswith('~'):
                        todas_proposicoes.add(efeito[1:].strip())
                    else:
                        todas_proposicoes.add(efeito.strip())
        
        self.para_numero = {}
        self.para_texto = {}
        
        for i, prop in enumerate(sorted(todas_proposicoes)):
            self.para_numero[prop] = i + 1
            self.para_texto[i + 1] = prop
        

    def _para_vetor(self, props, eh_final=False):
       
        vetor = []
        for prop in props:
            prop = prop.strip()
            if not prop:
                continue
            if prop.startswith('~'):    
                nome = prop[1:].strip()
                if nome in self.para_numero:
                    if eh_final:
                        vetor.append(-self.para_numero[nome])
                    else:
                        pass
            else:
                if prop in self.para_numero:
                    vetor.append(self.para_numero[prop])
        return sorted(vetor)
    
    def _preparar_acoes(self):
    
        acoes_prep = []
        
        for nome_acao, dados in self.acoes.items():
            pre_int = []
            for pre in dados['pre']:
                p = pre.strip()
                if not p:
                    continue
                if p.startswith('~'):
                    nome = p[1:].strip()
                    if nome in self.para_numero:
                        pre_int.append(-self.para_numero[nome])
                else:
                    if p in self.para_numero:
                        pre_int.append(self.para_numero[p])
            
            add_int = []
            del_int = []
            for efeito in dados['efeitos']:
                e = efeito.strip()
                if not e:
                    continue
                if e.startswith('~'):
                    nome = e[1:].strip()
                    if nome in self.para_numero:
                        del_int.append(self.para_numero[nome])
                else:
                    if e in self.para_numero:
                        add_int.append(self.para_numero[e])
            
            acoes_prep.append({
                'nome': nome_acao,
                'pre': pre_int,
                'add': add_int,
                'del': del_int
            })
        
        return acoes_prep
    
    # Converte estado (lista de inteiros positivos) para string legível
    def _estado_para_string(self, estado_int):
        
        partes = []
        for num in sorted(estado_int):
            if num > 0:
                partes.append(self.para_texto[num])
            else:
                partes.append(f"~{self.para_texto[abs(num)]}")
        return ';'.join(partes)
    
    # Verifica se um estado satisfaz o estado final
    def estado_eh_objetivo(self, estado_int):
        
        estado_set = set(estado_int)
        for objetivo in self.estado_final_int:
            if objetivo > 0:
                if objetivo not in estado_set:
                    return False
            else:
                if abs(objetivo) in estado_set:
                    return False
        return True
    
    # Gera (acao_nome, novo_estado_lista) a partir do estado_int.
    def gerar_sucessores(self, estado_int):

        sucessores = []
        estado_arvore = SortedSet(estado_int)  # árvore balanceada
        
        for acao in self.acoes_preparadas:
            aplicavel = True
            for pre in acao['pre']:
                if pre > 0:
                    if pre not in estado_arvore:
                        aplicavel = False
                        break
                else:
                    if abs(pre) in estado_arvore:
                        aplicavel = False
                        break
            if not aplicavel:
                continue
            
            # copia da árvore (O(n))
            novo_estado = SortedSet(estado_arvore)

            # remoções: O(log n)
            for d in acao['del']:
                novo_estado.discard(d)

            # adições: O(log n)
            for a in acao['add']:
                novo_estado.add(a)

            # já vem ordenado
            novo_estado_lista = list(novo_estado)

            sucessores.append((acao['nome'], novo_estado_lista))
        
        return sucessores

# ALGORITMOS DE BUSCA 

def busca_largura(mundo):
    print("[BFS] Iniciando busca em largura...")
    inicio = time.time()
    nos_expandidos = 0
    
    estado_inicial = mundo.estado_inicial_int
    if mundo.estado_eh_objetivo(estado_inicial):
        print("[BFS] Estado inicial já é objetivo!")
        return [], time.time() - inicio, 1
    
    fila = deque()
    fila.append((estado_inicial, []))
    visitados = set()
    visitados.add(tuple(estado_inicial))
    
    while fila:
        estado_atual, caminho = fila.popleft()
        nos_expandidos += 1
        
        sucessores = mundo.gerar_sucessores(estado_atual)
        for acao, novo_estado in sucessores:
            estado_tupla = tuple(novo_estado)
            if estado_tupla not in visitados:
                novo_caminho = caminho + [acao]
                if mundo.estado_eh_objetivo(novo_estado):
                    tempo = time.time() - inicio
                    print(f"[BFS] Solução encontrada! Nós expandidos: {nos_expandidos}, Tempo: {tempo:.3f}s")
                    return novo_caminho, tempo, nos_expandidos
                fila.append((novo_estado, novo_caminho))
                visitados.add(estado_tupla)
    tempo = time.time() - inicio
    print(f"[BFS] Nenhuma solução encontrada. Nós: {nos_expandidos}, Tempo: {tempo:.3f}s")
    return None, tempo, nos_expandidos

def busca_aestrela(mundo):
    print("[A*] Iniciando busca A* com heurística h_max...")
    inicio = time.time()
    nos_expandidos = 0

    def h_max(estado):
        # Estado inicial da heurística: custo 0 para fatos verdadeiros
        custo = {}
        fronteira = []

        for p in mundo.para_numero.values():
            if p in estado:
                custo[p] = 0
                fronteira.append(p)
            else:
                custo[p] = float('inf')

        # Relaxação: aplicar ações sem deletar nada
        mudou = True
        while mudou:
            mudou = False
            for acao in mundo.acoes_preparadas:
                # testar se precondições são alcançáveis
                pre_custos = []
                ok = True
                for pre in acao['pre']:
                    if pre > 0:
                        if custo[pre] == float('inf'):
                            ok = False
                            break
                        pre_custos.append(custo[pre])
                    else:
                        # pré-condição negativa ignorada na relaxação
                        pass
                if not ok:
                    continue

                custo_acao = 0 if not pre_custos else max(pre_custos)

                for a in acao['add']:
                    if custo[a] > custo_acao + 1:
                        custo[a] = custo_acao + 1
                        mudou = True

        # custo para atingir todos os objetivos
        h = 0
        for objetivo in mundo.estado_final_int:
            if objetivo > 0:
                h = max(h, custo[objetivo])
            else:
                # meta negativa: ignorada na relaxação
                pass

        return h if h != float('inf') else 999999

    estado_inicial = mundo.estado_inicial_int
    h_inicial = h_max(estado_inicial)

    fronteira = []
    heapq.heappush(fronteira, (h_inicial, 0, tuple(estado_inicial), []))
    g_melhor = {tuple(estado_inicial): 0}
    fechados = set()

    while fronteira:
        f, g, estado_tupla, caminho = heapq.heappop(fronteira)

        if estado_tupla in fechados:
            continue

        estado = list(estado_tupla)
        fechados.add(estado_tupla)
        nos_expandidos += 1

        if mundo.estado_eh_objetivo(estado):
            tempo = time.time() - inicio
            print(f"[A*] Solução encontrada! Nós expandidos: {nos_expandidos}, Tempo: {tempo:.3f}s")
            return caminho, tempo, nos_expandidos

        for acao, novo_estado in mundo.gerar_sucessores(estado):
            novo_tupla = tuple(novo_estado)
            novo_g = g + 1

            if novo_tupla not in g_melhor or novo_g < g_melhor[novo_tupla]:
                g_melhor[novo_tupla] = novo_g
                h = h_max(novo_estado)
                f_score = novo_g + h
                heapq.heappush(fronteira, (f_score, novo_g, novo_tupla, caminho + [acao]))

    tempo = time.time() - inicio
    print(f"[A*] Nenhuma solução encontrada. Nós: {nos_expandidos}, Tempo: {tempo:.3f}s")
    return None, tempo, nos_expandidos

def busca_profundidade_iterativa(mundo, limite_profundidade=20):
    print("[IDS] Iniciando busca em profundidade iterativa...")
    inicio = time.time()
    total_nos = 0
    
    sys.setrecursionlimit(10000)
    
    def busca_profundidade_limitada(estado, caminho, profundidade, limite, visitados):
        
        if profundidade > limite:
            return None
        
        estado_tupla = tuple(estado)
        if estado_tupla in visitados:
            return None
        
        visitados.add(estado_tupla)
        
        if mundo.estado_eh_objetivo(estado):
            return caminho
        
        for acao, novo_estado in mundo.gerar_sucessores(estado):
            resultado = busca_profundidade_limitada(
                novo_estado, caminho + [acao], profundidade + 1, limite, visitados
            )
            if resultado is not None:
                return resultado
        return None
    
    for profundidade in range(limite_profundidade + 1):
        print(f"[IDS] Testando profundidade {profundidade}...")
        resultado = busca_profundidade_limitada(mundo.estado_inicial_int, [], 0, profundidade, set())
        total_nos += 1
        if resultado is not None:
            tempo = time.time() - inicio
            print(f"[IDS] Solução encontrada! Profundidade: {profundidade}, Tempo: {tempo:.3f}s")
            return resultado, tempo, total_nos
    tempo = time.time() - inicio
    print(f"[IDS] Nenhuma solução encontrada. Tempo: {tempo:.3f}s")
    return None, time.time() - inicio, total_nos

# ==================== INTERFACE PRINCIPAL ====================

def carregar_e_resolver(arquivo_strips, algoritmo='aestrela'):
    print(f"\n{'='*80}")
    print(f" PROCESSANDO: {arquivo_strips}")
    print(f"{'='*80}")
    
    try:
        with open(arquivo_strips, 'r', encoding='utf-8') as f:
            conteudo = f.read()
    except FileNotFoundError:
        print(f" Arquivo não encontrado: {arquivo_strips}")
        return None
    except Exception as e:
        print(f" Erro ao ler arquivo: {e}")
        return None
    
    print("Criando representação do problema")
    mundo = MundoBlocosCorreto(conteudo)

    # INICIAR MEDICAO DE MEMORIA
    tracemalloc.start()

    # EXECUTAR ALGORITMO
    if algoritmo == 'bfs':
        solucao, tempo, nos = busca_largura(mundo)
    elif algoritmo == 'aestrela':
        solucao, tempo, nos = busca_aestrela(mundo)
    elif algoritmo == 'ids':
        solucao, tempo, nos = busca_profundidade_iterativa(mundo)
    elif algoritmo == 'bidirecional':
        solucao, tempo, nos = busca_bidirecional(mundo)
    else:
        print(f" Algoritmo desconhecido: {algoritmo}")
        tracemalloc.stop()
        return None

    # MEDICAO DA MEMORIA
    mem_atual, mem_pico = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    
    print(f"\n Memória atual: {mem_atual / (1024*1024):.2f} MB")
    print(f" Pico de memória: {mem_pico / (1024*1024):.2f} MB")

    if solucao:
        print(f"\n SOLUÇÃO ENCONTRADA!")
        print(f" Tempo: {tempo:.3f} segundos")
        print(f" Nos expandidos: {nos}")
        print(f" Numero de ações: {len(solucao)}")
        if len(solucao) <= 80:
            print(f"\n  Sequência completa:")
            for i, acao in enumerate(solucao, 1):
                print(f"     {i:3d}. {acao}")
        else:
            print(f"\n Primeiras 10 acoes:")
            for i, acao in enumerate(solucao[:10], 1):
                print(f"     {i:3d}. {acao}")
            print(f" ... {len(solucao)-10} acoees")
        return {
            'arquivo': arquivo_strips,
            'solucao': solucao,
            'tempo': tempo,
            'nos': nos,
            'acoes': len(solucao)
        }
    else:
        print(f"\n NAO ENCONTROU SOLUCAO")
        print(f" Tempo: {tempo:.3f} segundos")
        print(f" Nos expandidos: {nos}")
        return {
            'arquivo': arquivo_strips,
            'solucao': None,
            'tempo': tempo,
            'nos': nos,
            'acoes': 0
        }

def busca_bidirecional(mundo):
  
    print("[BIDIRECIONAL] Iniciando busca bidirecional BFS-BFS...")
    inicio = time.time()
    nos_expandidos = 0
    
    estado_inicial = mundo.estado_inicial_int
    estado_inicial_set = frozenset(estado_inicial)
    
    # Verificar se estado inicial já é objetivo
    if mundo.estado_eh_objetivo(estado_inicial):
        print("[BIDIRECIONAL] Estado inicial já é objetivo!")
        return [], time.time() - inicio, 1
    
    
    print("[BIDIRECIONAL] Buscando estado objetivo para iniciar busca reversa...")
    
    # Busca BFS para encontrar um estado objetivo
    fronteira_objetivo = deque()
    fronteira_objetivo.append((estado_inicial_set, []))
    visitados_objetivo = {estado_inicial_set}
    
    estado_objetivo_encontrado = None
    caminho_para_objetivo = None
    
    while fronteira_objetivo and estado_objetivo_encontrado is None:
        estado_atual, caminho_atual = fronteira_objetivo.popleft()
        
        # Verificar se é objetivo
        if mundo.estado_eh_objetivo(list(estado_atual)):
            estado_objetivo_encontrado = estado_atual
            caminho_para_objetivo = caminho_atual
            break
        
        # Expandir
        for acao, novo_estado in mundo.gerar_sucessores(list(estado_atual)):
            novo_set = frozenset(novo_estado)
            if novo_set not in visitados_objetivo:
                visitados_objetivo.add(novo_set)
                fronteira_objetivo.append((novo_set, caminho_atual + [acao]))
    
    if estado_objetivo_encontrado is None:
        print("[BIDIRECIONAL] Não foi possível encontrar um estado objetivo")
        tempo = time.time() - inicio
        return None, tempo, 0
    
    print(f"[BIDIRECIONAL] Estado objetivo encontrado (proposições: {len(estado_objetivo_encontrado)})")
    
    # ---------- SEGUNDA ETAPA: Busca bidirecional ----------
    # Agora temos:
    # - estado_inicial_set: ponto de partida da busca para frente
    # - estado_objetivo_encontrado: ponto de partida da busca para trás
    
    # Função para gerar predecessores (busca reversa)
    def gerar_predecessores(estado_set):

        predecessores = []
        estado_lista = list(estado_set)
        
        # Para cada ação, verificar se poderia ter sido aplicada para chegar neste estado
        for acao_info in mundo.acoes_preparadas:
            # Condições para que esta ação possa ter gerado o estado atual:
            # 1. Todos os "adds" da ação devem estar no estado atual
            # 2. Nenhum dos "dels" da ação pode estar no estado atual
            # 3. As pré-condições da ação devem ser satisfeitas pelo estado predecessor
            
            # Verificar adds e dels
            adds_presentes = all(a in estado_set for a in acao_info['add'])
            dels_ausentes = all(d not in estado_set for d in acao_info['del'])
            
            if not (adds_presentes and dels_ausentes):
                continue
            
            # Construir estado predecessor (antes da ação)
            estado_anterior = set(estado_set)
            
            # Remover os adds (no estado anterior eles não existiam ainda)
            for a in acao_info['add']:
                estado_anterior.discard(a)
            
            # Adicionar os dels (no estado anterior eles existiam)
            for d in acao_info['del']:
                estado_anterior.add(d)
            
            # Verificar se as pré-condições da ação são satisfeitas no estado anterior
            precond_ok = True
            for pre in acao_info['pre']:
                if pre > 0:
                    # Pré-condição positiva deve estar no estado anterior
                    if pre not in estado_anterior:
                        precond_ok = False
                        break
                else:
                    # Pré-condição negada (negativa) NÃO deve estar no estado anterior
                    if abs(pre) in estado_anterior:
                        precond_ok = False
                        break
            
            if precond_ok:
                predecessores.append((acao_info['nome'], frozenset(estado_anterior)))
        
        return predecessores
    
    # Inicializar as duas buscas
    frente_fronteira = deque()  # Fila para busca do início para objetivo
    frente_fronteira.append((estado_inicial_set, []))
    frente_visitados = {estado_inicial_set: []}  # estado -> caminho do início até ele
    
    tras_fronteira = deque()    # Fila para busca do objetivo para início
    tras_fronteira.append((estado_objetivo_encontrado, []))
    tras_visitados = {estado_objetivo_encontrado: []}  # estado -> caminho do objetivo até ele
    
    encontro_estado = None
    caminho_frente = None
    caminho_tras = None
    
    # Alternar entre as duas buscas
    iteracao = 0
    while frente_fronteira and tras_fronteira:
        iteracao += 1
        
        # ----- Expansão da busca para FRENTE (início -> objetivo) -----
        frente_tamanho = len(frente_fronteira)
        for _ in range(frente_tamanho):
            if not frente_fronteira:
                break
                
            estado_frente, caminho_f = frente_fronteira.popleft()
            nos_expandidos += 1
            
            # Verificar se este estado foi visitado pela busca reversa
            if estado_frente in tras_visitados:
                encontro_estado = estado_frente
                caminho_frente = caminho_f
                caminho_tras = tras_visitados[estado_frente]
                break
            
            # Expandir sucessores (busca normal)
            for acao, novo_estado in mundo.gerar_sucessores(list(estado_frente)):
                novo_set = frozenset(novo_estado)
                if novo_set not in frente_visitados:
                    frente_visitados[novo_set] = caminho_f + [acao]
                    frente_fronteira.append((novo_set, caminho_f + [acao]))
        
        if encontro_estado:
            break
        
        # ----- Expansão da busca para TRÁS (objetivo -> início) -----
        tras_tamanho = len(tras_fronteira)
        for _ in range(tras_tamanho):
            if not tras_fronteira:
                break
                
            estado_tras, caminho_t = tras_fronteira.popleft()
            nos_expandidos += 1
            
            # Verificar se este estado foi visitado pela busca para frente
            if estado_tras in frente_visitados:
                encontro_estado = estado_tras
                caminho_frente = frente_visitados[estado_tras]
                caminho_tras = caminho_t
                break
            
            # Expandir predecessores (busca reversa)
            for acao, estado_anterior in gerar_predecessores(estado_tras):
                if estado_anterior not in tras_visitados:
                    # Na busca reversa, a ação é aplicada no sentido contrário
                    # Então adicionamos a ação no início do caminho
                    tras_visitados[estado_anterior] = [acao] + caminho_t
                    tras_fronteira.append((estado_anterior, [acao] + caminho_t))
        
        if encontro_estado:
            break
        
        # Status a cada 1000 nós expandidos
        if nos_expandidos % 1000 == 0:
            print(f"[BIDIRECIONAL] Nós expandidos: {nos_expandidos}, "
                  f"Frente: {len(frente_fronteira)}, Trás: {len(tras_fronteira)}")
    
    # ---------- CONSTRUIR SOLUÇÃO COMPLETA ----------
    if encontro_estado:
        # Juntar os dois caminhos:
        # - caminho_frente: do início até o estado de encontro
        # - caminho_tras: do estado de encontro até o objetivo (mas na ordem reversa)
        
        # O caminho_tras foi construído do objetivo para o encontro,
        # então precisamos inverter a ordem das ações
        caminho_tras_invertido = list(reversed(caminho_tras))
        
        # Solução completa: início -> encontro -> objetivo
        solucao_completa = caminho_frente + caminho_tras_invertido
        
        tempo = time.time() - inicio
        print(f"[BIDIRECIONAL] Solução encontrada!")
        print(f"[BIDIRECIONAL] Estado de encontro com {len(encontro_estado)} proposições")
        print(f"[BIDIRECIONAL] Nós expandidos: {nos_expandidos}, Tempo: {tempo:.3f}s")
        print(f"[BIDIRECIONAL] Comprimento da solução: {len(solucao_completa)} ações")
        
        # Verificar se a solução é válida (opcional)
        print("[BIDIRECIONAL] Verificando solução...")
        estado_atual = estado_inicial_set
        valido = True
        
        for i, acao_nome in enumerate(solucao_completa):
            # Encontrar a ação
            acao_info = None
            for a in mundo.acoes_preparadas:
                if a['nome'] == acao_nome:
                    acao_info = a
                    break
            
            if not acao_info:
                print(f"[BIDIRECIONAL] Erro: Ação '{acao_nome}' não encontrada")
                valido = False
                break
            
            # Verificar pré-condições
            estado_lista = list(estado_atual)
            for pre in acao_info['pre']:
                if pre > 0:
                    if pre not in estado_atual:
                        print(f"[BIDIRECIONAL] Erro: Pré-condição {pre} não satisfeita")
                        valido = False
                        break
                else:
                    if abs(pre) in estado_atual:
                        print(f"[BIDIRECIONAL] Erro: Pré-condição negada {pre} não satisfeita")
                        valido = False
                        break
            
            if not valido:
                break
            
            # Aplicar ação
            novo_estado = set(estado_atual)
            for d in acao_info['del']:
                novo_estado.discard(d)
            for a in acao_info['add']:
                novo_estado.add(a)
            estado_atual = frozenset(novo_estado)
        
        if valido and mundo.estado_eh_objetivo(list(estado_atual)):
            print("[BIDIRECIONAL] Solução verificada com sucesso!")
        else:
            print("[BIDIRECIONAL] Aviso: Solução não verificada completamente")
        
        return solucao_completa, tempo, nos_expandidos
    
    tempo = time.time() - inicio
    print(f"[BIDIRECIONAL] Nenhuma solução encontrada.")
    print(f"[BIDIRECIONAL] Nós expandidos: {nos_expandidos}, Tempo: {tempo:.3f}s")
    print(f"[BIDIRECIONAL] Estados visitados na frente: {len(frente_visitados)}")
    print(f"[BIDIRECIONAL] Estados visitados atrás: {len(tras_visitados)}")
    
    return None, tempo, nos_expandidos


def menu_principal():
    print(" PLANEJADOR CORRIGIDO - MUNDO DOS BLOCOS")
    print("=" * 80)
    
    arquivos = [f for f in os.listdir('problemas') if f.endswith('.strips')]
    arquivos.sort()
    
    print(f"\n Encontrados {len(arquivos)} arquivo(s):")
    for i, arquivo in enumerate(arquivos, 1):
        print(f"  {i:2d}. {arquivo}")
    
    print(f"\n{'='*80}")
    print(" 1. Testar um arquivo específico")
   
    opcao = 1
    resultados = []

    if opcao == 1:
        print(f"\nArquivos disponíveis:")
        for i, arquivo in enumerate(arquivos, 1):
            print(f"{i:2d}. {arquivo}")
        escolha = input(f"\nEscolha (1-{len(arquivos)}): ").strip()
        try:
            idx = int(escolha) - 1
            if 0 <= idx < len(arquivos):
                arquivo = arquivos[idx]
                print(f"\nAlgoritmos disponíveis:")
                print("1. Busca em Largura (BFS)")
                print("2. A* (recomendado)")
                print("3. Busca em Profundidade Iterativa (IDS)")
                print("4. Busca Bidirecional (BFS-BFS)")
                algo_escolha = input("\nEscolha o algoritmo: ").strip()
                if algo_escolha == '1':
                    algoritmo = 'bfs'
                elif algo_escolha == '2':
                    algoritmo = 'aestrela'
                elif algo_escolha == '3':
                    algoritmo = 'ids'
                elif algo_escolha == '4':
                    algoritmo = 'bidirecional'
                caminho = f'problemas/{arquivo}'
                resultado = carregar_e_resolver(caminho, algoritmo)
                if resultado:
                    resultados.append(resultado)
            else:
                print(" Número inválido!")
        except:
            print(" Entrada inválida!")
    
    if resultados:
        salvar_resultados(resultados)

def salvar_resultados(resultados):
    try:
        with open('resultados_finais.txt', 'w', encoding='utf-8') as f:
            f.write("=" * 80 + "\n")
            f.write("RESULTADOS FINAIS - PLANEJADOR MUNDO DOS BLOCOS\n")
            f.write("=" * 80 + "\n\n")
            for res in resultados:
                f.write(f" ARQUIVO: {res['arquivo']}\n")
                if res['solucao']:
                    f.write(f" SOLUCAO ENCONTRADA\n")
                    f.write(f" Tempo: {res['tempo']:.3f} segundos\n")
                    f.write(f" Nos expandidos: {res['nos']}\n")
                    f.write(f" Acoes: {res['acoes']}\n\n")
                    f.write(" ACOES:\n")
                    for i, acao in enumerate(res['solucao'], 1):
                        f.write(f" {i:3d}. {acao}\n")
                else:
                    f.write(f" NENHUMA SOLUÇÃO ENCONTRADA\n")
                    f.write(f" Tempo: {res['tempo']:.3f} segundos\n")
                    f.write(f" Nós expandidos: {res['nos']}\n")
                f.write("\n" + "-" * 80 + "\n\n")
        print(f"\n Resultados salvos em 'resultados_finais.txt'")
    except Exception as e:
        print(f"Não consegui salvar resultados: {e}")

if __name__ == "__main__":
    menu_principal()
       