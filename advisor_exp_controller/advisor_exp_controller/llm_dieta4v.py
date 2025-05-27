import os
from langchain_neo4j import Neo4jGraph
from langchain_core.prompts import PromptTemplate, FewShotPromptTemplate, ChatPromptTemplate, MessagesPlaceholder

from langchain_openai import ChatOpenAI
from langchain_groq import ChatGroq
from langchain_core.output_parsers import StrOutputParser

from langgraph.checkpoint.memory import MemorySaver
from langgraph.graph import START, StateGraph
from langchain_core.messages import HumanMessage, AIMessage, trim_messages
from neo4j.exceptions import Neo4jError

from langchain_core.messages import BaseMessage
from langgraph.graph.message import add_messages
from typing_extensions import Annotated, TypedDict
from typing import Sequence, Literal
from datetime import datetime
import locale

class GeneralState(TypedDict):
        question: str
        messages_guardrails: Annotated[Sequence[BaseMessage], add_messages]
        messages_chat: Annotated[Sequence[BaseMessage], add_messages]
        query: str
        output: str
        next_action: str
        execute_query: bool
        relevant: bool
        update_output: bool

class Explainability():

    def __init__(self):

        os.environ["NEO4J_URI"] = "bolt://192.168.0.111:7687"
        os.environ["NEO4J_USERNAME"] = "neo4j"
        os.environ["NEO4J_PASSWORD"] = "robolabs"

        os.environ["GROQ_API_KEY"]
        os.environ["OPENAI_API_KEY"]

        oggi = datetime.now()
        #locale.setlocale(locale.LC_TIME, "it_IT.UTF-8")
        self.giorno = oggi.strftime("%a")
        timestamp = oggi.strftime("%Y-%m-%d_%H-%M-%S")
        os.makedirs("Conversazioni", exist_ok=True)
        self.nomefile="Conversazioni/conversazione_"+timestamp+".txt"

        llm_chat = ChatOpenAI(model="gpt-4o", temperature=0,)#GPT-4o  gpt-3.5-turbo
        llm = ChatGroq(model="llama-3.3-70b-versatile", temperature=0)#llama3-70b-8192
      
        examples = [
            {
                "question": "Cosa può mangiare rosario oggi?",
                "query": '''
MATCH (r:Ricetta), (p:Persona {{nome:'rosario'}})
OPTIONAL MATCH(p)-[:ha_mangiato]->(r2:Ricetta)
WITH collect(r2.nome) as ricette_mangiate, r, p
OPTIONAL MATCH(p)-[:ha_intolleranza]->(i:Intolleranza)-[:vieta]->(r2:Ricetta)
WITH collect(r2.nome) as ricette_vietate, ricette_mangiate, r, p, coalesce(i.nome,'nessuna') as intolleranza
MATCH (p)-[:ha_patologia]->(pat:Patologia)-[:segue]->(s:StileAlimentare)
OPTIONAL MATCH (p)-[:ha_mangiato {{giorno:'mercoledì', tipo_pasto:'pranzo'}}]->(r2:Ricetta)     //controllo se oggi ho pranzato
WITH  s.calorie_giornaliere - coalesce(r2.calorie,0) as calorie_rimaste, ricette_vietate, ricette_mangiate, r, s, intolleranza, pat, p
MATCH (r)-[c:contiene]->(m:Macronutriente)<-[v:vincola]-(s)
WITH collect(c.grammi<=v.grammi_max_pasto) as condizioni, r.nome as nome, calorie_rimaste, ricette_vietate, ricette_mangiate, r, intolleranza, pat, p
WHERE ALL(condizione IN condizioni WHERE condizione=true)
    AND NOT nome IN ricette_mangiate
    AND NOT nome IN ricette_vietate
    AND  r.calorie <= calorie_rimaste
WITH condizioni as condizioni_proteine_grassi_carboidrati_zuccheri, calorie_rimaste, ricette_vietate, ricette_mangiate, intolleranza, pat.nome as patologia, r.calorie as calorie_ricetta, p.nome as persona, nome as ricetta_consigliata order by rand()
RETURN * limit 1
        '''
            },
            {
                "question": "quali ricette può mangiare una persona intollerante al glutine?",
                "query": '''
MATCH (r:Ricetta) 
OPTIONAL MATCH (i:Intolleranza {{nome:'glutine'}})-[:vieta]->(r2:Ricetta) 
with collect(r2.nome) as ricette_vietate, r
WHERE NOT r.nome IN ricette_vietate
RETURN collect(r.nome) as ricette_consentite
        ''',
            },
                {
        "question": "quali ricette può mangiare una persona diabetica?",
        "query": '''MATCH (r:Ricetta)-[c:contiene]->(m:Macronutriente)<-[v:vincola]-(s:StileAlimentare)<-[se:segue]-(p:Patologia {{nome:'diabete'}})
WITH collect(c.grammi<=v.grammi_max_pasto) as condizioni, r.nome as nome
WHERE ALL(condizione IN condizioni WHERE condizione=true)
RETURN collect(nome) as ricette_consentite''',
            },
            {
                "question": "rosario può mangiare la mozzarella in carrozza?",
                "query" : '''
MATCH (r:Ricetta {{nome: 'mozzarella in carrozza'}}), (p:Persona {{nome:'rosario'}})
OPTIONAL MATCH(p)-[:ha_mangiato]->(r2:Ricetta)
WITH collect(r2.nome) as ricette_mangiate, r, p
OPTIONAL MATCH(p)-[:ha_intolleranza]->(i:Intolleranza)-[:vieta]->(r2:Ricetta)
WITH collect(r2.nome) as ricette_vietate, ricette_mangiate, r, p, coalesce(i.nome,'nessuna') as intolleranza
MATCH (p)-[:ha_patologia]->(pat:Patologia)-[:segue]->(s:StileAlimentare)
OPTIONAL MATCH (p)-[:ha_mangiato {{giorno:'mercoledì', tipo_pasto:'pranzo'}}]->(r2:Ricetta)     //controllo se oggi ho pranzato
WITH  s.calorie_giornaliere - coalesce(r2.calorie,0) as calorie_rimaste, ricette_vietate, ricette_mangiate, r, s, intolleranza, pat,coalesce(r2.calorie,0) as calorie_pranzo, p
MATCH (r)-[c:contiene]->(m:Macronutriente)<-[v:vincola]-(s)
WITH collect(c.grammi<=v.grammi_max_pasto) as condizioni, r.nome as nome, calorie_rimaste, ricette_vietate, ricette_mangiate, r, intolleranza, pat, calorie_pranzo, p
WITH
    CASE
            WHEN ALL(condizione IN condizioni WHERE condizione=true)
                AND NOT nome IN ricette_mangiate
                AND NOT nome IN ricette_vietate
                AND  r.calorie <= calorie_rimaste
                THEN true
            ELSE false
    END as consentita, calorie_rimaste, calorie_pranzo, ricette_vietate, ricette_mangiate, condizioni as condizioni_proteine_grassi_carboidrati_zuccheri, r.calorie as calorie_ricetta, intolleranza, pat.nome as patologia, p.nome as persona
RETURN *
        '''
            },
            {
                "question": "sono rosario, dammi un'alternativa al salmone grigliato",
                "query": '''
MATCH (r:Ricetta), (p:Persona {{nome:'rosario'}}), (r3:Ricetta {{nome:'salmone grigliato'}})
OPTIONAL MATCH(p)-[:ha_mangiato]->(r2:Ricetta)
WITH collect(r2.nome) as ricette_mangiate, r, p, r3.nome as ricetta_non_voluta
OPTIONAL MATCH(p)-[:ha_intolleranza]->(i:Intolleranza)-[:vieta]->(r2:Ricetta)
WITH collect(r2.nome) as ricette_vietate, ricette_mangiate, r, p, coalesce(i.nome,'nessuna') as intolleranza, ricetta_non_voluta
MATCH (p)-[:ha_patologia]->(pat:Patologia)-[:segue]->(s:StileAlimentare)
OPTIONAL MATCH (p)-[:ha_mangiato {{giorno:'mercoledì', tipo_pasto:'pranzo'}}]->(r2:Ricetta)     //controllo se oggi ho pranzato
WITH  s.calorie_giornaliere - coalesce(r2.calorie,0) as calorie_rimaste, ricette_vietate, ricette_mangiate, r, s, intolleranza, pat, p, ricetta_non_voluta
MATCH (r)-[c:contiene]->(m:Macronutriente)<-[v:vincola]-(s)
WITH collect(c.grammi<=v.grammi_max_pasto) as condizioni, r.nome as nome, calorie_rimaste, ricette_vietate, ricette_mangiate, r, intolleranza, pat, p, ricetta_non_voluta
WHERE ALL(condizione IN condizioni WHERE condizione=true)
    AND NOT nome IN ricette_mangiate
    AND NOT nome IN ricette_vietate
    AND  r.calorie <= calorie_rimaste
    AND nome <> ricetta_non_voluta
WITH condizioni as condizioni_proteine_grassi_carboidrati_zuccheri, calorie_rimaste, ricette_vietate, ricette_mangiate, intolleranza, pat.nome as patologia, r.calorie as calorie_ricetta, p.nome as persona, nome as ricetta_consigliata order by rand()
RETURN * LIMIT 1
        '''
            }
        ]

        guardrails_system= f'''
Sei un robot di nome Pepper che assiste una persona per l'alimentazione, e quindi per ricette da consigliare, cosa poter mangiare, valutando intolleranze patologie e stili alimentari o diete. Utilizzi come base di conoscenza un database Neo4j che interroghi con le query Cypher.
Devi verificare le seguenti condizioni:
1) La domanda posta è pertinente allo scenario corrente, rappresentato dallo schema del grafo?
2) Attenendoti esclusivamente alle informazioni che possiedi già in memoria, puoi rispondere alla domanda? Concentrati solo sulle informazioni che ti servono per rispondere, non alla risposta della domanda.
Rispondi ad entrambe esclusivamente con "si" o "no"  (esempio :si no)
'''

        guardrails_prompt = ChatPromptTemplate.from_messages(
            [
                (
                    "system",
                    guardrails_system,
                ),
                MessagesPlaceholder(variable_name="messages")
            ]
        )

        self.guardrails_chain = guardrails_prompt | llm_chat | StrOutputParser()

        example_prompt_template = PromptTemplate.from_template(
            '''Domanda: {question}\nQuery: {query}'''
        )

        generate_query_prompt = FewShotPromptTemplate(
            examples = examples,
            example_prompt  = example_prompt_template,
            prefix = '''
Tu sei un Robot di nome Pepper, esperto in Neo4j. Data una domanda in input crea una query Cypher sintatticamente corretta da eseguire.
Qui trovi lo schema con le informazioni del database neo4j:
{schema}.
Sotto trovi un numero di esempi di domande e la loro corrispettiva query Cypher.''',
    suffix = '''
Oggi è '''+self.giorno+'''. Ritornami esclusivamente la query da eseguire e non aggiungere altro testo. Rispondi in italiano.
{prec_res}
Domanda: {question}
Query: 
        ''',
            input_variables = ["question", "schema","prec_res"],
        )

        self.generate_query_chain = generate_query_prompt | llm | StrOutputParser()

        final_answer_system=f'''
Sei un robot di nome Pepper che assiste una persona per l'alimentazione e utilizzi come base di conoscenza un database Neo4j che interroghi con le query Cypher.
Questa persona ti fa delle domande e per rispondere ricavi le informazioni dal database, rielaborandole in un discorso di senso compiuto, spiegando i motivi della risoposta e il ragionamento che hai utilizzato.
Non salutare in ogni risposta, solo nel primo messaggio o quando cambia l'utente
'''

        generate_final_answer_prompt = ChatPromptTemplate.from_messages(
            [
                (
                    "system",
                    final_answer_system,
                ),
                MessagesPlaceholder(variable_name="messages"),
            ]
        )

        self.generate_final_answer_chain = generate_final_answer_prompt | llm_chat | StrOutputParser()

        self.trimmer = trim_messages(
            max_tokens=4000,
            strategy="last",
            token_counter=llm_chat,
            include_system=True,
            allow_partial=False,
            start_on="human",
        )

        workflow = StateGraph(GeneralState)

        workflow.add_node(self.guardrails)
        workflow.add_node(self.generate_query)
        workflow.add_node(self.generate_final_answer)

        workflow.add_edge(START, "guardrails")
        workflow.add_conditional_edges(
            "guardrails",
            self.guardrails_condition,
        )
        workflow.add_edge("generate_query", "generate_final_answer")

        memory = MemorySaver()
        self.app = workflow.compile(checkpointer=memory)

        self.graph = Neo4jGraph()
        self.schema=self.graph.schema

        self.config1 = {"configurable": {"thread_id": "paziente"}}

    def guardrails(self, state: GeneralState):

        state.setdefault('output', 'nessuno')
        question = state.get("question")

        message = f'''
Schema:
{self.schema}

Output precedente:
{state.get("output")}

Domanda: {question}
'''

        trimmed_messages = self.trimmer.invoke(state["messages_guardrails"])
        input_messages = trimmed_messages + [HumanMessage(message)]
        conditions = self.guardrails_chain.invoke({"messages": input_messages})
        print(conditions)

        if conditions[0:2] == 'no':
            relevant = False
            execute_query = False
            next_action = "generate_final_answer"

        else:
            relevant = True

            if conditions[3:5]  == 'no':
                execute_query = True 
                next_action = "generate_query"
            
            else: 
                execute_query = False 
                next_action = "generate_final_answer"

        return {
            "next_action" : next_action,
            "execute_query" : execute_query,
            "relevant" : relevant,
            "messages_guardrails" : [HumanMessage(message), AIMessage(conditions)],
        }

    def guardrails_condition(self, state: GeneralState):#  -> Literal["generate_final_answer", "generate_query"]:
        next_action = state.get("next_action")
        return next_action

    def generate_query(self, state: GeneralState):

        state.setdefault('output', 'nessuno')
        question = state.get("question")
        query = self.generate_query_chain.invoke({"question": question, "schema": self.schema , "prec_res": "Output query precedente: "+ state.get("output")})
        
        print("\033[1;32mQuery\n\033[0;32m"+query+"\n\033[0m")

        try:
            output=str(self.graph.query(query))
            update_output = output!='[]'
        except Neo4jError as e:
            print("\n\033[1;31mErrore nella query: \033[0;31m"+str(e)+"\n\033[0m")
            output = state.get("output")
            update_output = False
        
        print("\033[1;32mOutput:\033\n[0;32m"+output+"\n\033[0m")

        return{
            "output" : output,
            "query" : query,
            "upadate_output" : update_output
        }

    def generate_final_answer(self, state: GeneralState):

        question=state.get("question")
        trimmed_messages = self.trimmer.invoke(state["messages_chat"])

        if not state.get("relevant"):
            answer = "Grazie per la tua domanda, ma non risulta pertinente allo scenario attuale."
        
        elif not state.get("output"):
            answer = "Purtroppo non so la risposta, prova a riformulare la domanda."
        
        elif state.get("execute_query"):
            question = f'''
Data una query, creami un discorso che spieghi i vari controlli che effettua e infine la conclusione a cui sei arrivato. Per ogni step della query fornisci anche il risultato, considerando il suo output, e infine dai un resoconto.
Query: {state.get("query")}
Output: {state.get("output")}
Domanda: {state.get("question")}
Fornisci una risposta sintetica e discorsiva come se la dovesse pronunciare il robot a voce al paziente in questione, quindi senza elenchi puntati, senza dettagli tecnici sul database e simulando una conversazione.
'''
            input_messages = trimmed_messages + [HumanMessage(question)]
            answer = self.generate_final_answer_chain.invoke({"messages" : input_messages})
            
        else:
            question = f'''
Attenendoti esclusivamente agli output precedenti, rispondi alla seguente domanda in modo esaustivo spiegando e motivando il perché della risposta e il ragionamento che segui per rispondere.
Domanda: {state.get("question")}
Fornisci una risposta sintetica e discorsiva come se la dovesse pronunciare il robot a voce al paziente in questione, quindi senza elenchi puntati e senza dettagli tecnici sul database e simulando una conversazione.
'''
            input_messages = trimmed_messages + [HumanMessage(question)]
            answer = self.generate_final_answer_chain.invoke({"messages" : input_messages})
        
        return{
            "messages_chat" : [HumanMessage(question), AIMessage(answer)]
        }

    def chat(self,question):

        with open(self.nomefile, "w", encoding="utf-8") as file:

            file.write(f"Domanda: {question}\n")
            response = self.app.invoke({"question": question}, self.config1)

            answer = response["messages_chat"][-1].content
            print(answer)

            execute_query = '(query non fatta)' if not response["execute_query"] else ''
            file.write(f"Risposta: {answer} {execute_query}\n\n")

        return answer