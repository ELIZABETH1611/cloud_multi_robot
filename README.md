# multi_robots

# if rank>number_rooms-1
# ejecuta todo lo que corresponde al robot

# if rank<number_rooms:
# ejecuta los procesos de entrenamiento en el CLOUD

# Al inicio el robot tiene una copia antigua de la red
# usa esa copia para elejir una acción en función de su "STATE"
# luego envia todos los datos a el CLOUD
# el cloud recibe los datos, los entrena y envia al robot una copia de los nuevos pesos
# el robot actualiza su red y repite el proceso


# ese es mas o menos el caso que tienen ahí, la idea es que el CLOUD entrene
# y cuando reciba un "state" del robot escoja la acción y envie la acción  al robot
# por lo tanto la parte de enviar los pesos desde cloud al robot
# y que el robot lo actualice no lo tendrán que hacer
