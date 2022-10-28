# multi_robots
Hay un archivo que se llama many_robots,con en ese archivo creo los multiples robots y targets
lo tengo así porque le asigno un nombre diferente a cada robot y a cada target
#####################
Dentro del código "main (AHORA ES EL PRINCIPAL)" encontrarán:

if rank>number_rooms-1  ejecuta todo lo que corresponde al robot

if rank<number_rooms: ejecuta los procesos de entrenamiento en la nube

# Cómo funciona:

Al inicio el robot tiene una copia antigua de la red, misma que usa para escoger
una acción en función de su "STATE". Luego envia todos los datos de memory a la nube

Por otro lado la nube recibe los datos de las memorias, los entrena.
Cada cierto tiempo envia al robot una copia de los nuevos pesos

Una vez el robot actualiza su red, se repite el proceso de elegir acción y enviar la información

#####################

Ese es mas o menos el caso que tienen ahí, la idea es que la nube entrene los datos
y cuando reciba un mensaje que contenga el "state" del robot escoja la acción. Para luego enviar la acción  al robot

Por lo tanto la parte de enviar los pesos desde la nube al robot y que el robot lo actualice no lo tendrán que hacer
